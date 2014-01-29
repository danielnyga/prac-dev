# PRAC -- MODULE FOR WORD SENSE DISAMBIGUATION
#
# (C) 2011-2014 by Daniel Nyga (nyga@cs.uni-bremen.de)
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
from prac.core import PRACModule, PRACPIPE, PRACKnowledgeBase, DB_TRANSFORM
from mln.mln import readMLNFromFile
import os
from mln.database import readDBFromFile, Database
import logging
from mln.methods import ParameterLearningMeasures
import sys
from prac.inference import PRACInferenceStep
from prac.wordnet import WordNet

class PRACWSD(PRACModule):
    '''
    PRAC reasoning module for performing very basic word sense disambiguation.
    '''
    
    def initialize(self): 
        log = logging.getLogger(self.__class__.__name__)
        self.learned_mln = None
        self.mln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'wsd.mln'), grammar='PRACGrammar', logic='FuzzyLogic')
    
    
    def get_known_concepts(self):
        if self.mln is None:
            raise Exception('Load an MLN first.')
        return self.mln.domains['concept']
        
    
    @DB_TRANSFORM
    def get_most_probable_senses(self, db):
        log = logging.getLogger(self.__class__.__name__)
        log.info(self.mln.domains)
        mrf = self.mln.groundMRF(db, method='FastConjunctionGrounding')
#         result = mrf.inferEnumerationAsk(['has_sense'], None,verbose=True,shortOutput=True, useMultiCPU=True)
        result = mrf.inferWCSP(['has_sense'], None,verbose=True,shortOutput=True, useMultiCPU=True)
#         result = mrf.inferMCSAT(['has_sense'], None,verbose=True,shortOutput=True, useMultiCPU=True)       
        db_ = Database(self.mln)
        for atom, truth in result.iteritems():
            if truth == 1:
                db_.addGroundAtom(str(atom))
        return db_
        
    
    @PRACPIPE
    def infer(self, pracinference): 
        log = logging.getLogger(self.name)
        inf_step = PRACInferenceStep(pracinference, self)
        mt = self.load_pracmt('wsd')
        mln = mt.learned_mln
#         mln.addConstant('concept', 'NULL')
#         mln.addConstant('sense', 'Nullsense')
        for input_db in pracinference.get_inference_steps_of_module('wn_senses').output_dbs:
            evidence = input_db.duplicate(mln)
#             input_db.printEvidence()
#             for s in input_db.query('has_sense(?w, ?s) ^ is_a(?s, ?c)', truthThreshold=1):
#                 if s['?s'] == 'Nullsense': continue
#                 if not s['?c'] in mln.domains['concept']: continue
#                 is_a = 'is_a(%s,%s)' % (s['?s'], s['?c'])
#                 has_sense = 'has_sense(%s,%s)' % (s['?w'], s['?s'])
#                 if is_a in input_db.evidence:
#                     evidence.addGroundAtom(is_a, input_db.evidence[is_a])
#                 if has_sense in input_db.evidence:
#                     evidence.addGroundAtom(has_sense, input_db.evidence[has_sense])
# #             self.prac.getModuleByName('wn_senses').addPossibleWordSensesToDBs(evidence)
#             evidence.printEvidence()
            inf_step.input_dbs.append(evidence)
            mrf = mln.groundMRF(evidence)
            result = mrf.inferEnumerationAsk(['has_sense', 'action_role'], None,verbose=True,shortOutput=True)
#             result.writeResult(sys.stdout)
        return inf_step


    def load_mln(self, path):
        self.mln = readMLNFromFile(path, logic='FuzzyLogic', grammar='PRACGrammar')

    
    @PRACPIPE
    def train(self, praclearn): 
        dbs = readDBFromFile(self.mln, os.path.join(self.module_path, 'mln', 'training.db'))
        wn = self.prac.getModuleByName('wn_senses')
        wn.addFuzzyEvidenceToDBs(*dbs)
        for db in dbs:
            db.printEvidence() 
        kb = WSD(self, 'wsd')
        kb.train(dbs)
        kb.learned_mln.write(open(os.path.join(self.module_path, 'mln', 'learned.mln'), 'w+'))
        self.save_pracmt(kb)
        

class WSD(PRACKnowledgeBase):
    
    def train(self, training_dbs):
        log = logging.getLogger(self.__class__.__name__)
        mln = self.module.mln
        self.training_dbs = training_dbs
        mln.write(sys.stdout, color=True)
        queryPreds = list(mln.predicates)
        queryPreds.remove('is_a')
        params = {}
        params['optimizer'] = 'bfgs'
        params['gaussianPriorSigma'] = 1.
        self.learned_mln = mln.learnWeights(training_dbs, ParameterLearningMeasures.BPLL_CG, **params)
        self.wn = WordNet(self.learned_mln.domains['concept'] + [])
        self.learned_mln.write(sys.stdout, color=True)
        
