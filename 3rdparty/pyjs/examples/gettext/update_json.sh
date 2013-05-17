#!/bin/bash
PYJSBASE="../.."

cd lang
for f in *.po ; do
	b=`basename $f .po`
	../${PYJSBASE}/contrib/mo2json.py $b.mo
	cp $b.json ../public/lang/
done
