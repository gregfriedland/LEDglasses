#!python

from math import pow, ceil
import sys

GAMMA = float(sys.argv[1])

print "PROGMEM prog_uchar lookupGamma[] = {"

for i in range(256):
    result = ceil(pow(i/255.0, GAMMA)*255.0 + 0.5)
    print "%d," % result,
print
