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

from subprocess import Popen, PIPE, STDOUT
from nltk.corpus import wordnet as wn


MLN = None
def doXval(file_list_ukb,file_list_truth):
    regex_ctx = re.compile("ctx_(\d+)\s+(w\d)\s+(\d+-\w)")
    
    for i in range(0,len(file_list_ukb)):
        file = open(file_list_ukb[i],'r')
        dbs_as_textfiles = file.read()
        dbs = readDBFromFile(MLN,file_list_truth[i])
        cm = ConfusionMatrix()
        
        cmd = './ukb_wsd -D wnet30_dict.txt -K wn30+gloss.bin --ppr {}'.format(file_list_ukb[i])
        p = Popen(cmd, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
        ukb_result_regex = regex_ctx.findall(p.stdout.read())
        ukb_result_dict = {}
        
        for res in ukb_result_regex:
            ID = res[2]
            synset = wn._synset_from_pos_and_offset(str(ID[-1:]), int(ID[:8]))
            if res[0] in ukb_result_dict:
                ukb_result_dict[res[0]][res[1]] = str(synset.name())
            else:
                ukb_result_dict[res[0]] = {res[1]: str(synset.name())} 
        
        for key, val in ukb_result_dict.iteritems():
            db = dbs[int(key)-1]
            
            
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
            
            
            if "w2" in val.keys():
                cm.addClassificationResult(dobj_sense,val["w2"])
            
            if "w1" in val.keys():
                cm.addClassificationResult(predicate_sense,val["w1"])
        
        cm.toFile(os.path.join("ukb_result",os.path.basename(file_list_ukb[i])))
           
        
        
        
        
    
if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 3:
        input_dir_ukb = args[0]
        input_dir_truth = args[1]
        MLN = readMLNFromFile(args[2], grammar='PRACGrammar', logic='FirstOrderLogic')
        
        file_list_ukb = []
        file_list_truth = []
        
        for filename in os.listdir(input_dir_ukb):
            file_list_ukb.append(os.path.join(input_dir_ukb,filename))
            
        for filename in os.listdir(input_dir_truth):
            file_list_truth.append(os.path.join(input_dir_truth,filename))
        
        doXval(file_list_ukb,file_list_truth)
            