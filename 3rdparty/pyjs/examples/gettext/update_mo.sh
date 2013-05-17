#!/bin/bash

cd lang
for f in *.po ; do
	b=`basename $f .po`
	msgfmt -o $b.mo $f
done
