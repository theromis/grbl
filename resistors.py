from __future__ import division
import itertools

list = [1, 1.2, 2.2, 3.3, 4.7, 5.6, 7.5, 8.2, 10, 15, 22, 27, 33, 39, 47, 56, 68, 75, 82, 100,
    120, 150, 180, 220, 270, 330, 390, 470, 510, 680, 820, 1000, 1200, 1500, 2200, 2700, 3300,
    3900, 4700, 5600, 6800, 7500, 8200, 10000, 15000, 20000, 22000, 33000, 39000, 47000, 51000,
    56000, 68000, 75000, 82000, 100000, 150000, 180000, 220000, 330000, 470000, 560000, 680000, 1000000]
out = [ (a/b, a, b) for a, b in itertools.permutations(list, 2)]
for c, a, b in sorted(out):
    print "{:10.4f}".format(abs(c-4)),  "{:10.4f}".format(c), a, b
