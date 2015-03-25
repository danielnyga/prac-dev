#!/bin/bash
echo "Started senses xval"          
python xvalACSensesRole.py -k 10 -m has_sense sense Template.mln Data.db 
wait $!
echo "Finished senses xval"
echo "Started roles xval"
python xvalACSensesRole.py -k 10 -m action_role role Template.mln Data.db
wait $!
echo "Finished roles xval"
