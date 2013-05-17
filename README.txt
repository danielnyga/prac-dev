PROBABILISTIC ROBOT ACTION CORES
================================


INSTALLATION INSTRUCTIONS:
==========================
- get ProbCog:
  $ git clone https://github.com/opcode81/ProbCog.git
  and follow the installation instructions
  
- get Toulbar2 from mulcyber.toulouse.inra.fr/projects/toulbar2/
  and make sure it's in your PATH

- get NLTK and WordNet:
  $ sudo apt-get install python-nltk python-jpype python-pyparsing python-tk python-webpy pyjamas-pyjs
  
  There should be a dialog popping up. Go to the "Corpora" tab and
  tick the WordNet corpus. Hit download.

- <PATH-TO-PRAC> $ python make_apps.py
- <PATH-TO-PRAC> $ source env.sh

- test with:
  $ pracinfer Flipping "Flip the pancake around."
  $ pracinfer Flipping "Flip the burgers."
  $ pracinfer Flipping "Flip the pancake with a spatula."
  $ pracinfer Flipping "Flip with a spatula."
  
- you can relearn the models with:
  $ praclearn Flipping
