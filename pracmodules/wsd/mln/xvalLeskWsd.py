# Markov Logic Networks -- Automated Cross-Validation Tool
#
# (C) 2012 by Daniel Nyga (nyga@cs.tum.edu)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import time
import os
import sys
import traceback

from optparse import OptionParser
from mln.mln import readMLNFromFile,Predicate
from mln.database import readDBFromFile, Database
from random import shuffle, sample
import math
from mln.methods import LearningMethods, InferenceMethods
from wcsp.converter import WCSPConverter
from utils.eval import ConfusionMatrix
from mln.util import strFormula, mergeDomains
from multiprocessing import Pool
from utils.clustering import SAHN, Cluster, computeClosestCluster
import logging
import praclog
from logging import FileHandler
from prac.core import PRAC, PRACKnowledgeBase
from prac.wordnet import WordNet
from prac.inference import PRACInference
import re
import pickle
from nltk.wsd import lesk

MLN = None
def doXval(file_list):
    regex_new_db = re.compile("\n\s*-+\s*\n")

    for file_path in file_list:
        file = open(file_path,'r')
        dbs_as_textfiles = regex_new_db.split(file.read())
        dbs = readDBFromFile(MLN,file_path)
        cm = ConfusionMatrix()
        
        for i in range(0,len(dbs_as_textfiles)):
            sentence = dbs_as_textfiles[i].strip().split("\n")[0]
            db = dbs[i]
            
            predicate_name = None
            predicate_sense = None
            dobj_name = None
            dobj_sense = None 
            
            for q1 in db.query('predicate(?w)'):
                predicate_name = '-'.join(q1['?w'].split('-')[:-1])# extract everything except the number (e.g. compound words like heart-shaped from heart-shaped-4)
                for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                    predicate_sense = q2['?s']
                
                for q2 in db.query('dobj({},?s)'.format(q1['?w'])):
                    
                    for q3 in db.query('has_sense({},?s)'.format(q2['?s'])):
                        dobj_sense = q3['?s']
                    
                    dobj_name = '-'.join(q2['?s'].split('-')[:-1])# extract everything except the number (e.g. compound words like heart-shaped from heart-shaped-4)
                        
            splited_sentence = sentence.split(" ")
            
            
            pred = lesk(splited_sentence,dobj_name)
            if pred is not None:
                cm.addClassificationResult(dobj_sense,pred.name())
            
            pred = lesk(splited_sentence,predicate_name)
            if pred is not None:
                cm.addClassificationResult(predicate_sense,pred.name())
        
        cm.toFile(os.path.join("lesk_result",os.path.basename(file_path)))
            
        
        
        
        
    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FirstOrderLogic')
        
        file_list = []
        
        for filename in os.listdir(input_dir):
            file_list.append(os.path.join(input_dir,filename))
        
        doXval(file_list)
            