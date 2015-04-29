#!/bin/bash
echo "Started senses xval"
path=$(pwd)          
python xvalACSensesRole.py -k 10 -m -f $path/Senses-Result has_sense sense Template.mln Data.db 
wait $!
echo "Finished senses xval"
echo "Started roles xval"
python xvalACSensesRole.py -k 10 -m -f $path/Roles-Result action_role role Template.mln Data.db
wait $!
echo "Finished roles xval"
