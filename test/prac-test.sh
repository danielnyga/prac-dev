#!/bin/bash

if [ "$1" = "local" ]
then
    PRACMLNREPO=$PRACMLN_HOME
    PRACREPO=$PRAC_HOME
elif [ "$1" = "remote" ]
then
	PRACMLNREPO=git@github.com:danielnyga/pracmln-dev
	PRACREPO=git@github.com:danielnyga/prac-dev
elif [ "$1" = "uncommitted" ]
then
	echo
else
	echo specify either "local", "remote" or "uncommitted"
	exit -1
fi


cd $PRAC_HOME/test
mkdir src
if [ "$1" = "uncommitted" ]
then
	echo copying uncommitted state of local repo...
	rsync -qa $PRACMLN_HOME src --exclude test
	rsync -qa $PRAC_HOME src --exclude test
else
	echo cloning pracmln temporarily from $PRACMLNREPO
	git clone $PRACMLNREPO src
	echo cloning prac temporarily from $PRACREPO
	git clone $PRACREPO src
fi
docker build -t prac/test .
echo Removing temporary files...
rm -rf src