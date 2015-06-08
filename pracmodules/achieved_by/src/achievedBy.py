# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2015 by Daniel Nyga (nyga@cs.tum.edu)
# (C) 2015 by Sebastian Koralewski (seba@informatik.uni-bremen.de)
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

from prac.core import PRACModule, PRACPIPE, PRACKnowledgeBase, PRAC
import logging
from mln import readMLNFromFile, readDBFromFile#, MLNParsingError
from mln.methods import LearningMethods
import sys
from wcsp.converter import WCSPConverter
from mln.database import Database
import os
from prac.inference import PRACInferenceStep, PRACInference
from mln.util import mergeDomains
from utils import colorize
from pracutils import printListAndTick
from prac.wordnet import WordNet
import yaml


class AchievedBy(PRACModule):
    '''
    
    '''
    
    def initialize(self):
        pass
    
    def shutdown(self):
        pass
    
    def extendDBWithAchievedByEvidence(self,db,queryMln):
        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
        acDomain = db.mln.domains.get("actioncore")
        acDomain.extend(queryMln.domains.get("actioncore"))
        acDomain = set(acDomain)
        db_ = Database(queryMln)
        
        for ac1 in acDomain:
            for ac2 in acDomain:
                if ac1 == actioncore: continue
                db_.addGroundAtom("achieved_by({},{})".format(ac1,ac2),0)
        
        for atom, truth in sorted(db.evidence.iteritems()):
            db_.addGroundAtom(atom,truth)
        
        return db_
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RECOGNIZING ACHIEVED BY  ' , (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)
        
        kb = params.get('kb', None)
        if kb is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
            dbs = kb.dbs
        
        inf_step = PRACInferenceStep(pracinference, self)
        
        for db in dbs:
            
            for q in db.query('action_core(?w,?ac)'):
                #running = True
                #This list is used to avoid an infinite loop during the achieved by inference.
                #To avoid this infinite loop, the list contains the pracmlns which were inferenced during the process.
                #Every pracmln should be used only once during the process because the evidence for the inference will always remain the same.
                #So if the pracmln hadnt inferenced a plan in the first time, it will never do it.
                
                db_ = Database(db.mln)
                #Need to remove possible achieved_by predicates from previous achieved_by inferences
                for atom, truth in sorted(db.evidence.iteritems()):
                    if 'achieved_by' in atom: continue
                    db_.addGroundAtom(atom,truth)
                    
                wordnet = WordNet(concepts=None)
                actionword = q['?w']
                actioncore = q['?ac']

                if kb is None:
                    print 'Loading Markov Logic Network: %s' % colorize(actioncore, (None, 'white', True), True)
                    useKB = self.load_pracmt(actioncore)
                    
                else:
                    useKB = kb
                
                concepts = useKB.query_mln.domains.get('concept', [])
                
                for q in db_.query("has_sense(?w,?s)"):
                    for concept in concepts:
                        sim = wordnet.wup_similarity(q["?s"], concept)
                        db_.addGroundAtom('is_a(%s,%s)' % (q["?s"], concept),sim)
                
                #Inference achieved_by predicate        
                #self.kbs.append(useKB)  
                #params.update(useKB.query_params)
                db_ = self.extendDBWithAchievedByEvidence(db_,useKB.query_mln)
                result_db = list(useKB.infer(db_))
                result_db_ = []
                #Removing achieved_by predicates with prob zero
                for r_db in result_db:
                    r_db_ = Database(useKB.query_mln)
                    for atom, truth in sorted(r_db.evidence.iteritems()):
                        _, predname, args = db.mln.logic.parseLiteral(atom)
                        if predname == 'achieved_by'  and truth == 0: continue
                        r_db_.addGroundAtom(atom,truth)
                    result_db_.append(r_db_)
                
                result_db = result_db_
                    
                for r_db in result_db:
                    for q in r_db.query('achieved_by({0},?nac) ^ action_core(?w,{0})'.format(actioncore)):
                        achievedByAc = q['?nac']
                        
                        r_db_ = Database(r_db.mln)
                        
                        for atom, truth in sorted(r_db.evidence.iteritems()):
                            if 'is_a' in atom: continue
                            r_db_.addGroundAtom(atom,truth)
                        
                        inf_step.output_dbs.append(r_db_)
                        print actionword + " achieved by: " + achievedByAc

                kb = useKB
                     
        if kb is not None:
            png, ratio = kb.get_cond_prob_png(filename=self.name)
            inf_step.png = (png, ratio)   
        return inf_step
    
