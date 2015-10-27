'''
Created on Dec 4, 2012

@author: nyga
'''
from prac_nltk.corpus import wordnet as wn
import sys
import prac_nltk
import os

prac_nltk.data.path = [os.path.join('.', 'data', 'nltk_data')]

if __name__ == '__main__':
    args = sys.argv[1:]
    print 
    if len(args) == 2:
        synset = wn.synset(args[0])
        concepts = set()
        for path in synset.hypernym_paths():
            concepts.update(map(lambda x: x.name, path))
        for c in concepts:
            print 'is_a(%s, %s)' % (args[1], c)
        print 
    if len(args) == 3 and args[0] == 'all':
        synsets = wn.synsets(args[1], args[2])
        for s in synsets:
            print  s.name + ": " + s.definition + ' (' + ';'.join(s.examples) + ')'
        print
    
    
