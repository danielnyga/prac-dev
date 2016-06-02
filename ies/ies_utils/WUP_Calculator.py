'''
Created on Nov 3, 2015

@author: seba
'''
from __future__ import division
from nltk.corpus import wordnet as wn
from mln.database import readDBFromFile
from mln.mln import readMLNFromFile,Predicate
from prac.wordnet import WordNet
from pracutils.pracgraphviz import render_gv
from utils.graphml import to_html_hex
from os.path import basename
from ies_models import Constants
from scipy import stats

import sys  
import os



if __name__ == '__main__':
    
    word_list = ['crimp.v.02',
'flip.v.01',
'pour.v.04',
'beat.v.04',
'swerve.v.01',
'flip.v.08',
'roll.v.01',
'whip.v.03',
'spread.v.02',
'move.v.03',
'assume.v.05',
'grab.v.03',
'arouse.v.06',
'tapdance.v.01',
'go.v.03',
'turn.v.01',
'separate.v.09',
'move_over.v.01',
'face.v.05',
'startle.v.02',
'edit.v.03',
'chop.v.02',
'throw.v.02',
'rotate.v.04',
'stir.v.02',
'pulsate.v.01',
'rim.v.03',
'grind.v.04',
'begin.v.02',
'turn_out.v.10',
'jump.v.01',
'turn.v.10']
    '''
    
    word_list = [ 'have.v.01',
'keep.v.03',
'store.v.02',
'save.v.02',
'cut.v.20',
'recruit.v.03',
'retain.v.03',
'wield.v.01',
'freeze.v.08',
'catch.v.25',
'score.v.03',
'film.v.01',
'photograph.v.01',
'book.v.03',
'reserve.v.01',
'bankrupt.v.01',
'reduce.v.08',
'cut.v.19'
]
'''
    wordnet = WordNet(concepts=None)
    reference_syn = "flip.v.08"
    print "Reference SYN: {}".format(reference_syn)
    for word in word_list:
        
        sim = wordnet.wup_similarity(word,reference_syn)
        print "WORD : {} | SIM : {} | MAX_CONFIDENCE : {} ".format(word,str(sim),str(stats.hmean([1.0,1.0,sim])))
        
        
