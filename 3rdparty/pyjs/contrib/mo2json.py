#!/usr/bin/env python
# Copyright (C) 2012 Kees Bos <cornelis.bos@gmail.com>
# LISENCE : Apache 2.0 or WTFPL, choose what suits your needs
#
# This tool is to be used with pyjamas.JSONTranslations
# Converts .mo files to .json files

import gettext
import re
import sys
try:
    # included in python 2.6...
    from json import dumps
except ImportError:
    # recommended library (python 2.5)
    from simplejson import dumps


def mo2json(fp):
    t = gettext.GNUTranslations(fp)

    plural_forms = None
    nplurals = 2
    header = {}
    for line in t._catalog[""].split("\n"):
        kv = [i.strip() for i in line.strip().split(':', 1)]
        if kv and len(kv) > 1:
            header[kv[0]] = kv[1]
            if kv[0].lower() == 'plural-forms':
                plural_forms = kv[1]
    if plural_forms is not None:
        m = re.search('nplurals *= *(\d+)',  plural_forms)
        if not m:
            raise ValueError("Invalid Plural-Forms")
        nplurals = int(m.group(1))
    c = {}
    for k, v in t._catalog.iteritems():
        if k != "":
            if not isinstance(k, tuple):
                lst = [v]
                c[k] = lst
            else:
                k, n = k
                if n < nplurals:
                    lst = c.get(k, list())
                    l = len(lst)
                    if l <= n:
                        lst = lst + [None] * (n-l+1)
                    lst[n] = v
                    c[k] = lst
    c[""] = header
    return dumps(c)

if __name__ == '__main__':
    if len(sys.argv) == 1:
        sys.stdout.write(mo2json(sys.stdin))
    else:
        for fname in sys.argv[1:]:
            json = mo2json(open(fname))
            if fname.endswith('.mo'):
                fname = fname[:-3]
            fname = fname + '.json'
            open(fname, 'w').write(json)
