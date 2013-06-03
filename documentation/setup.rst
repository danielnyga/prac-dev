======================
Installation and Setup
======================

#. Get *ProbCog*: ::

    $ git clone https://github.com/opcode81/ProbCog.git

  and follow the installation instructions.
#. Get *Toulbar2* from http://mulcyber.toulouse.inra.fr/projects/toulbar2/ and make sure it's in your ``$PATH``
#. Get the following packages: ::

    $ sudo apt-get install python-nltk python-jpype python-pyparsing python-tk python-webpy python-scipy
  
#. Execute the PRAC installation:::

    <PATH-TO-PRAC> $ python make_apps.py
    <PATH-TO-PRAC> $ source env.sh
  
#. Test your installation with: ::

    $ pracinfer Flipping "Flip the pancake around."
    $ pracinfer Flipping "Flip the burgers."
    $ pracinfer Flipping "Flip the pancake with a spatula."
    $ pracinfer Flipping "Flip with a spatula."
  
