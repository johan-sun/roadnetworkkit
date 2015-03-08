#!/usr/bin/env python
import sys
import re
from os import path
prefix = "." if len(sys.argv) == 1 else sys.argv[1]
for l in sys.stdin.readlines():
    l = l.strip()
    sl = re.split('%s+'%(path.sep), l)
    print>>sys.stdout, l
    print>>sys.stdout,prefix + '/%s/%s/%s'%tuple(sl[3:6])
