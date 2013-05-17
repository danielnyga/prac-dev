#!/bin/bash

for f in `ls lang/*.po` ; do
	xgettext --omit-header -j -o $f Gettext.py
done
