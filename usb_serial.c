/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl v0.9

  Copyright (c) 2012-2014 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/
/* 
  This file is based on work from Grbl v0.8, distributed under the 
  terms of the MIT-license. See COPYING for more details.  
    Copyright (c) 2009-2011 Simen Svale Skogsrud
    Copyright (c) 2011-2012 Sungeun K. Jeon
*/ 

#include <avr/interrupt.h>
#include "system.h"
#include "serial.h"
#include "motion_control.h"
#include "protocol.h"
#include "settings.h"

#define GET_STATUS          0
#define CLEAR_FEATURE           1
#define SET_FEATURE         3
#define SET_ADDRESS         5 
#define GET_DESCRIPTOR          6
#define GET_CONFIGURATION       8
#define SET_CONFIGURATION       9
#define GET_INTERFACE           10
#define SET_INTERFACE           11

#define CDC_SET_LINE_CODING     0x20
#define CDC_GET_LINE_CODING     0x21
#define CDC_SET_CONTROL_LINE_STATE  0x22

#if defined(__AVR_AT90USB162__)
#define HW_CONFIG() 
#define PLL_CONFIG() (PLLCSR = ((1<<PLLE)|(1<<PLLP0)))
#define USB_CONFIG() (USBCON = (1<<USBE))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_ATmega32U4__)
#define HW_CONFIG() (UHWCON = 0x01)
#define PLL_CONFIG() (PLLCSR = 0x12)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB646__)
#define HW_CONFIG() (UHWCON = 0x81)
#define PLL_CONFIG() (PLLCSR = 0x1A)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#elif defined(__AVR_AT90USB1286__)
#define HW_CONFIG() (UHWCON = 0x81)
#define PLL_CONFIG() (PLLCSR = 0x16)
#define USB_CONFIG() (USBCON = ((1<<USBE)|(1<<OTGPADE)))
#define USB_FREEZE() (USBCON = ((1<<USBE)|(1<<FRZCLK)))
#endif  

#if 0 
#define VENDOR_ID       0x16C0
#define PRODUCT_ID      0x047A
#define STR_SERIAL_NUMBER   L"12345"
#else
#define VENDOR_ID       0x2341
#define PRODUCT_ID      0x8041
#define STR_SERIAL_NUMBER   L"00"
#endif

#define STR_MANUFACTURER    L"Arduino LLC"
//#define STR_PRODUCT     L"Arduino Yun "
#define STR_PRODUCT     L"Arduino CNC "GRBL_VERSION"("GRBL_VERSION_BUILD")"

#define LSB(n) (n & 0xFF)
#define MSB(n) ((n >> 8) & 0xFF)

#define CDC_ACM_ENDPOINT    2
#define CDC_RX_ENDPOINT     3
#define CDC_TX_ENDPOINT     4

#define CDC_ACM_BUFFER      EP_SINGLE_BUFFER
#define CDC_RX_BUFFER       EP_DOUBLE_BUFFER
#define CDC_TX_BUFFER       EP_DOUBLE_BUFFER

#if defined(__AVR_AT90USB162__)
#define CDC_ACM_SIZE        16
#define CDC_RX_SIZE     32
#define CDC_TX_SIZE     32
#else
#define CDC_ACM_SIZE        16
#define CDC_RX_SIZE     64
#define CDC_TX_SIZE     64
#endif

#define EP_SINGLE_BUFFER        0x02
#define EP_DOUBLE_BUFFER        0x06
#define EP_SIZE(s)  ((s) == 64 ? 0x30 : \
        ((s) == 32 ? 0x20 : \
         ((s) == 16 ? 0x10 : \
          0x00)))

#define ENDPOINT0_SIZE      16


#define EP_TYPE_CONTROL         0x00

#define EP_TYPE_BULK_IN         0x81
#define EP_TYPE_BULK_OUT        0x80
#define EP_TYPE_INTERRUPT_IN        0xC1

#define TX_RX_LED_INIT  DDRD |= (1<<5), DDRB |= (1<<0)
#define TXLED0          PORTD |= (1<<5)
#define TXLED1          PORTD &= ~(1<<5)
#define RXLED0          PORTB |= (1<<0)
#define RXLED1          PORTB &= ~(1<<0)

#define TX_RX_LED_PULSE_MS 100

/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

// Descriptors are the data that your computer reads when it auto-detects
// this USB device (called "enumeration" in USB lingo).  The most commonly
// changed items are editable at the top of this file.  Changing things
// in here should only be done by those who've read chapter 9 of the USB
// spec and relevant portions of any USB class specifications!

static const uint8_t PROGMEM device_descriptor[] = {
    18,                                 // bLength
    1,                                  // bDescriptorType
    0x00, 0x02,                         // bcdUSB
    2,                                  // bDeviceClass
    0,                                  // bDeviceSubClass
    0,                                  // bDeviceProtocol
    ENDPOINT0_SIZE,                     // bMaxPacketSize0
    LSB(VENDOR_ID), MSB(VENDOR_ID),     // idVendor
    LSB(PRODUCT_ID), MSB(PRODUCT_ID),   // idProduct
    0x00, 0x01,                         // bcdDevice
    1,                                  // iManufacturer
    2,                                  // iProduct
    0,                                  // iSerialNumber
    1                                   // bNumConfigurations
};

#define CONFIG1_DESC_SIZE (9+8+9+5+5+4+5+7+9+7+7)
static const uint8_t PROGMEM config1_descriptor[CONFIG1_DESC_SIZE] = {
    // configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
    9,                                  // bLength;
    2,                                  // bDescriptorType;
    LSB(CONFIG1_DESC_SIZE),             // wTotalLength
    MSB(CONFIG1_DESC_SIZE),
    2,                                  // bNumInterfaces
    1,                                  // bConfigurationValue
    0,                                  // iConfiguration
    0x80,                               // bmAttributes (Bus powered)
    0xFA,                               // bMaxPower (500ma/2) = 0xFA
    // Abstract Control Management
    8,                                  // bLength
    0x0B,                               // bDescriptorType
    0,                                  // bFirstInterface
    2,                                  // bInterfaceCount
    2,                                  // bFunctionClass
    2,                                  // bFunctionSubClass
    1,                                  // bFunctionProtocol
    0,                                  // iFunction
    // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
    9,                                  // bLength
    4,                                  // bDescriptorType
    0,                                  // bInterfaceNumber
    0,                                  // bAlternateSetting
    1,                                  // bNumEndpoints
    0x02,                               // bInterfaceClass
    0x02,                               // bInterfaceSubClass
    0x00,                               // bInterfaceProtocol
    0,                                  // iInterface
    // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
    5,                                  // bFunctionLength
    0x24,                               // bDescriptorType
    0x00,                               // bDescriptorSubtype
    0x10, 0x01,                         // bcdCDC
    // Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
    5,                                  // bFunctionLength
    0x24,                               // bDescriptorType
    0x01,                               // bDescriptorSubtype
    0x01,                               // bmCapabilities
    1,                                  // bDataInterface
    // Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
    4,                                  // bFunctionLength
    0x24,                               // bDescriptorType
    0x02,                               // bDescriptorSubtype
    0x06,                               // bmCapabilities
    // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
    5,                                  // bFunctionLength
    0x24,                               // bDescriptorType
    0x06,                               // bDescriptorSubtype
    0,                                  // bMasterInterface
    1,                                  // bSlaveInterface0
    // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
    7,                                  // bLength
    5,                                  // bDescriptorType
    CDC_ACM_ENDPOINT | 0x80,            // bEndpointAddress
    0x03,                               // bmAttributes (0x03=intr)
    CDC_ACM_SIZE, 0,                    // wMaxPacketSize
    64,                                 // bInterval
    // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
    9,                                  // bLength
    4,                                  // bDescriptorType
    1,                                  // bInterfaceNumber
    0,                                  // bAlternateSetting
    2,                                  // bNumEndpoints
    0x0A,                               // bInterfaceClass
    0x00,                               // bInterfaceSubClass
    0x00,                               // bInterfaceProtocol
    0,                                  // iInterface
    // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
    7,                                  // bLength
    5,                                  // bDescriptorType
    CDC_RX_ENDPOINT,                    // bEndpointAddress
    0x02,                               // bmAttributes (0x02=bulk)
    CDC_RX_SIZE, 0,                     // wMaxPacketSize
    0,                                  // bInterval
    // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
    7,                                  // bLength
    5,                                  // bDescriptorType
    CDC_TX_ENDPOINT | 0x80,             // bEndpointAddress
    0x02,                               // bmAttributes (0x02=bulk)
    CDC_TX_SIZE, 0,                     // wMaxPacketSize
    0,                                   // bInterval
};

static const uint8_t PROGMEM endpoint_config_table[] = {
    0,
    1, EP_TYPE_INTERRUPT_IN,  EP_SIZE(CDC_ACM_SIZE) | CDC_ACM_BUFFER,
    1, EP_TYPE_BULK_OUT,      EP_SIZE(CDC_RX_SIZE) | CDC_RX_BUFFER,
    1, EP_TYPE_BULK_IN,       EP_SIZE(CDC_TX_SIZE) | CDC_TX_BUFFER
};

// If you're desperate for a little extra code memory, these strings
// can be completely removed if iManufacturer, iProduct, iSerialNumber
// in the device desciptor are changed to zeros.
struct usb_string_descriptor_struct {
    uint8_t bLength;
    uint8_t bDescriptorType;
    int16_t wString[];
};
static const struct usb_string_descriptor_struct PROGMEM string0 = {
    4,
    3,
    {0x0409}
};
static const struct usb_string_descriptor_struct PROGMEM string1 = {
    sizeof(STR_MANUFACTURER),
    3,
    STR_MANUFACTURER
};
static const struct usb_string_descriptor_struct PROGMEM string2 = {
    sizeof(STR_PRODUCT),
    3,
    STR_PRODUCT
};

#if 0
static const struct usb_string_descriptor_struct PROGMEM string3 = {
    sizeof(STR_SERIAL_NUMBER),
    3,
    STR_SERIAL_NUMBER
};
    {0x0303, 0x0409, (const uint8_t *)&string3, sizeof(STR_SERIAL_NUMBER)},
#endif

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
typedef struct _descriptor_list_struct {
    uint16_t    wValue;
    uint16_t    wIndex;
    const uint8_t   *addr;
    uint8_t     length;
} descriptor_list_struct_t;

static const descriptor_list_struct_t PROGMEM descriptor_list[] = {
    {0x0100, 0x0000, device_descriptor, sizeof(device_descriptor)},
    {0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor)},
    {0x0300, 0x0000, (const uint8_t *)&string0, 4},
    {0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER)},
    {0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT)},
    {0xFFFF, 0, NULL, 0},
};

// Returns the number of bytes used in the RX serial buffer.
uint8_t serial_get_rx_buffer_count()
{
    return 0;
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
    return 0;
}


static volatile uint8_t usb_configuration=0;
static uint8_t cdc_line_rtsdtr=0;

void
serial_init()
{
    usb_configuration = 0;
    cdc_line_rtsdtr = 0;
#if 1
    HW_CONFIG();
    USB_FREEZE();               // enable USB
    PLL_CONFIG();               // config PLL, 16 MHz xtal
    while (!(PLLCSR & (1<<PLOCK))) ;    // wait for PLL lock
    USB_CONFIG();               // start USB clock
#else
    UHWCON = (1 << UVREGE);
    USBCON = (1 << USBE);
    PLLCSR = (1 << PINDIV)|(1 << PLLE)|(1 << PLOCK);
    UECONX = (1 << STALLRQ)|(1 << EPEN);
    USBCON = (1 << USBE)|(1<<OTGPADE);
#endif

    UDCON = 0;              // enable attach resistor
    UDIEN = (1<<EORSTE)|(1<<SOFE);

    TX_RX_LED_INIT;
}


void
serial_reset_read_buffer() 
{
    uint8_t intr_state;

    if (usb_configuration) {
        intr_state = SREG;
        UENUM = CDC_RX_ENDPOINT;
        while ((UEINTX & (1<<RWAL))) {
            UEINTX = 0x6B;
        }
        SREG = intr_state;
    }
}

// Misc functions to wait for ready and send/receive packets
#define usb_wait_in_ready()    while (!(UEINTX & (1<<TXINI ))) ;
#define usb_wait_receive_out() while (!(UEINTX & (1<<RXOUTI))) ;
#define usb_wait_io() while (!(UEINTX & ((1<<TXINI)|(1<<RXOUTI)))) ;
#define usb_send_in() UEINTX = ~(1<<TXINI );
#define usb_ack_out() UEINTX = ~(1<<RXOUTI);

static volatile uint8_t transmit_flush_timer=0;
static uint8_t transmit_previous_timeout=0;

#define TRANSMIT_TIMEOUT    25   /* in milliseconds */
#define TRANSMIT_FLUSH_TIMEOUT  5   /* in milliseconds */
volatile uint8_t tx_led_pulse, rx_led_pulse;
// transmit a character.
void
serial_write(uint8_t c)
{
#if 1
    uint8_t timeout, intr_state;

    // if we're not online (enumerated and configured), error
    if (!usb_configuration)
        return;

    // interrupts are disabled so these functions can be
    // used from the main program or interrupt context,
    // even both in the same program!
    intr_state = SREG;
    UENUM = CDC_TX_ENDPOINT;

    // if we gave up due to timeout before, don't wait again
    if (transmit_previous_timeout) {
        if (!(UEINTX & (1<<RWAL))) {
            SREG = intr_state;
            return;
        }
        transmit_previous_timeout = 0;
    }
    // wait for the FIFO to be ready to accept data
    timeout = UDFNUML + TRANSMIT_TIMEOUT;

    // while we ready to transmit
    while (!(UEINTX & (1<<RWAL))) {
        SREG = intr_state;

        // have we waited too long?  This happens if the user
        // is not running an application that is listening
        if (UDFNUML >= timeout) {
            transmit_previous_timeout = 1;
            return;
        }

        // has the USB gone offline?
        if (!usb_configuration)
            return;

        // get ready to try checking again
        intr_state = SREG;
        UENUM = CDC_TX_ENDPOINT;
    }
    // actually write the byte into the FIFO
    UEDATX = c;
    // if this completed a packet, transmit it now!
    if (!(UEINTX & (1<<RWAL)))
        UEINTX = 0x3A;
    transmit_flush_timer = TRANSMIT_FLUSH_TIMEOUT;
    SREG = intr_state;
#else
    if (!usb_configuration)
        return;

    u8 timeout = 250;       // 250ms timeout on send? TODO
    while (len)
    {
        u8 n = USB_SendSpace(ep);
        if (n == 0)
        {
            if (!(--timeout))
                return -1;
            delay(1);
            continue;
        }

        if (n > len)
            n = len;
        {
            LockEP lock(ep);
            // Frame may have been released by the SOF interrupt handler
            if (!ReadWriteAllowed())
                continue;
            len -= n;
            if (ep & TRANSFER_ZERO)
            {
                while (n--)
                    Send8(0);
            }
            else if (ep & TRANSFER_PGM)
            {
                while (n--)
                    Send8(pgm_read_byte(data++));
            }
            else
            {
                while (n--)
                    Send8(*data++);
            }
            if (!ReadWriteAllowed() || ((len == 0) && (ep & TRANSFER_RELEASE))) // Release full buffer
                ReleaseTX();
        }
    }

#endif
    TXLED1;
    tx_led_pulse = TX_RX_LED_PULSE_MS;
}

// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t
serial_read()
{
    uint8_t intr_state, ret = 0;

    if (!usb_configuration)
        return ret;

    intr_state = SREG;
    UENUM = CDC_RX_ENDPOINT;

    usb_wait_io();
    if ((UEBCLX == 0)) { // Empty buffer => flush it
        UEINTX = 0x6B;
        goto exit;
    }

    // take one byte out of the buffer
    ret = UEDATX;

    RXLED1;
    rx_led_pulse = TX_RX_LED_PULSE_MS;
exit:
    SREG = intr_state;

    return ret;
}


/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/


// USB Device Interrupt - handle all device-level events
// the transmit buffer flushing is triggered by the start of frame
//
ISR(USB_GEN_vect)
{
    uint8_t intbits, t;

    intbits = UDINT;
    UDINT = 0;
    if (intbits & (1<<EORSTI)) {
        UENUM = 0;
        UECONX = 1;
        UECFG0X = EP_TYPE_CONTROL;
        UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
        UEIENX = (1<<RXSTPE);
        usb_configuration = 0;
        cdc_line_rtsdtr = 0;
    }

    if (usb_configuration && intbits & (1<<SOFI)) {
        t = transmit_flush_timer;
        if (t) {
            transmit_flush_timer = --t;
            if (!t) {
                UENUM = CDC_TX_ENDPOINT;
                UEINTX = 0x3A;
            }
        }
        // check whether the one-shot period has elapsed.  if so, turn off the LED
        if (tx_led_pulse && !(--tx_led_pulse))
            TXLED0;
        if (rx_led_pulse && !(--rx_led_pulse))
            RXLED0;

    }
}

static inline int
get_descriptor(uint16_t value, uint16_t index, uint16_t len)
{
    const uint8_t *buf;
    uint16_t idx, val;
    const uint8_t *desc_addr=NULL;
    uint8_t	desc_length=0;
    uint8_t i, n;

    for (buf = (uint8_t*)descriptor_list;
            (val = pgm_read_word(&buf[0])) != 0xFFFF;
            buf+=sizeof(*descriptor_list)) {

        idx = pgm_read_word(&buf[2]);
        if (val == value && idx == index) {
            desc_addr = (const uint8_t *)pgm_read_word(&buf[4]);
            desc_length = pgm_read_byte(&buf[6]);

            len = (len < 256) ? len : 255;
            if (len > desc_length) len = desc_length;
            do {
                // wait for host ready for IN packet
                usb_wait_io();
                // send IN packet
                n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
                for (i = n; i; i--) {
                    UEDATX = pgm_read_byte(desc_addr++);
                }
                len -= n;
                usb_send_in();
            } while (len || n == ENDPOINT0_SIZE);
            return 0;
        }
    }

    if (desc_addr == NULL) {
        UECONX = (1<<STALLRQ)|(1<<EPEN);  //stall
    }
    return 0;
}

static inline int
set_address(uint16_t value)
{
    usb_send_in();
    usb_wait_in_ready();
    UDADDR = value | (1<<ADDEN);
    return 0;
}

static inline int
set_configuration(uint16_t value)
{
    uint8_t i;
    const uint8_t *cfg;
    usb_configuration = value;
    cdc_line_rtsdtr = 0;
    transmit_flush_timer = 0;
    usb_send_in();
    cfg = endpoint_config_table;
    for (i=1; i<5; i++) {
        UENUM = i;
        uint8_t en = pgm_read_byte(cfg++);
        UECONX = en;
        if (en) {
            UECFG0X = pgm_read_byte(cfg++);
            UECFG1X = pgm_read_byte(cfg++);
        }
    }
    UERST = 0x1E;
    UERST = 0;
    return 0;
}

static inline int
get_configuration()
{
    usb_wait_in_ready();
    UEDATX = usb_configuration;
    usb_send_in();
    return 0;
}

// config line: dte_rate[4] = {0x0000e100(57600)}, char_format = 0, parity_type = 0, data_bits = 8
static uint8_t cdc_line_coding[]={0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x08};

static inline int
cdc_line_coding_get()
{
    uint8_t i, *p;
    usb_wait_in_ready();
    p = cdc_line_coding;
    for (i=0; i<7; i++) {
        UEDATX = *p++;
    }
    usb_send_in();
    return 0;
}

static inline int
cdc_line_coding_set()
{
    uint8_t i, *p;
    usb_wait_receive_out();
    p = cdc_line_coding;
    for (i=0; i<7; i++) {
        *p++ = UEDATX;
    }
    usb_ack_out();
    usb_send_in();
    return 0;
}

static inline int
cdc_control_line_state_set(uint16_t value)
{
    cdc_line_rtsdtr = value;
    usb_wait_in_ready();
    usb_send_in();
    return 0;
}

static inline int
status_get(uint16_t index)
{
    uint8_t i;
    usb_wait_in_ready();
    i = 0;
#ifdef SUPPORT_ENDPOINT_HALT
    if (request_type == 0x82) {
        UENUM = index;
        if (UECONX & (1<<STALLRQ)) i = 1;
        UENUM = 0;
    }
#endif
    UEDATX = i;
    UEDATX = 0;
    usb_send_in();
    return 0;
}

#ifdef SUPPORT_ENDPOINT_HALT
static inline int
feature_set(uint16_t index)
{
    int i = index & 0x7F;
    if (i >= 1 && i <= MAX_ENDPOINT) {
        usb_send_in();
        UENUM = i;
        if (request == SET_FEATURE) {
            UECONX = (1<<STALLRQ)|(1<<EPEN);
        } else {
            UECONX = (1<<STALLRQC)|(1<<RSTDT)|(1<<EPEN);
            UERST = (1 << i);
            UERST = 0;
        }
    }
    return 0;
}
#endif

// USB Endpoint Interrupt - endpoint 0 is handled here.  The
// other endpoints are manipulated by the user-callable
// functions, and the start-of-frame interrupt.
//
ISR(USB_COM_vect)
{
    uint8_t intbits;
    uint8_t request_type;
    uint8_t request;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
    int ok = -1;


    UENUM = 0;
    intbits = UEINTX;
    if (intbits & (1<<RXSTPI)) {
        request_type = UEDATX;
        request = UEDATX;
        wValue = UEDATX;
        wValue |= (UEDATX << 8);
        wIndex = UEDATX;
        wIndex |= (UEDATX << 8);
        wLength = UEDATX;
        wLength |= (UEDATX << 8);

        UEINTX = ~((1<<RXSTPI) | (1<<RXOUTI) | (1<<TXINI));

        if (request == GET_DESCRIPTOR) {
            ok = get_descriptor(wValue, wIndex, wLength);
        } else if (request == SET_ADDRESS) {
            ok = set_address(wValue);
        } else if (request == GET_STATUS) {
            ok = status_get(wIndex);
        } else if (request == SET_CONFIGURATION          && request_type == 0) {
            ok = set_configuration(wValue);
        } else if (request == GET_CONFIGURATION          && request_type == 0x80) {
            ok = get_configuration();
        } else if (request == CDC_GET_LINE_CODING        && request_type == 0xA1) {
            ok = cdc_line_coding_get();
        } else if (request == CDC_SET_LINE_CODING        && request_type == 0x21) {
            ok = cdc_line_coding_set();
        } else if (request == CDC_SET_CONTROL_LINE_STATE && request_type == 0x21) {
            ok = cdc_control_line_state_set(wValue);
#ifdef SUPPORT_ENDPOINT_HALT
        } else if ((request == CLEAR_FEATURE || request == SET_FEATURE)
                && request_type == 0x02 && wValue == 0) {
            ok = feature_set(wIndex);
#endif
        }
    }
    if (ok == 0)
        UEINTX = ~(1<<TXINI);
    else
        UECONX = (1<<STALLRQ) | (1<<EPEN);	// stall
}
