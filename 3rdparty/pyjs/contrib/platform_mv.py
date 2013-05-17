#!/usr/bin/env python

platforms = [
'pysm',
'PyJS',
'PyV8',
'Mozilla',
'pywebkitdfb',
'pywebkitgtk',
'pyqt4',
'mshtml',
'hulahop']

import os
for r in open("fnames").readlines():
    r = r.strip()
    l = r.split("/")
    if not l[-2] == 'platform':
        continue
    l = l[:-2] + l[-1:]
    for p in platforms:
        if l[-1].endswith("%s.py" % p):
            fname = l[-1][:-len(p)-3]
            l[-1] = ("%s.%s.py" % (fname, p))
            fname = "/".join(l)
            os.system("git mv %s %s" % (r, fname))
