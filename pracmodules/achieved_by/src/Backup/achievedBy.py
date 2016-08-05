# PROBABILISTIC ROBOT ACTION CORES 
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

PRAC_HOME = os.environ['PRAC_HOME']
achievedByModulePath = os.path.join(PRAC_HOME, 'pracmodules', 'achieved_by')
planListFilePath = os.path.join(achievedByModulePath,'plan_list.yaml')

class AchievedBy(PRACModule):
    '''
    
    '''
    
    def initialize(self):
        pass
    
    def shutdown(self):
        pass
    
    def getPlanList(self):
        planListFile = open(planListFilePath, 'r')
        yamlData = yaml.load(planListFile)
        
        return yamlData['planList']
    
    def updateActionRoles(self,db,actionword,actioncore):
        #Init new PRAC instance for inference roles
        prac = PRAC()
        prac.wordnet = WordNet(concepts=None)
        infer = PRACInference(prac, 'None')
        senses = prac.module('senses_and_roles')
        senses.initialize()
        sensesKBFile = senses.load_prac_kb(actioncore)
        
        #Create senses and roles evidence DB
        db_senses = Database(sensesKBFile.query_mln)
        for atom, truth in sorted(db.evidence.iteritems()):
            if 'action_core' in atom or 'is_a' in atom or 'action_role' in atom: continue
            db_senses.addGroundAtom(atom,truth)
        db_senses.addGroundAtom('action_core('+actionword+","+actioncore+")")
        
        #Neccessary to avoid not defined attributes due to pickled object
        sensesKB = PRACKnowledgeBase(prac)
        sensesKB.filename = sensesKBFile.filename
        sensesKB.query_mln = sensesKBFile.query_mln
        sensesKB.query_mln_str = sensesKBFile.query_mln_str
        sensesKB.query_params = sensesKBFile.query_params
        sensesKB.dbs.append(db_senses)
   
        #start new role inference
        prac.run(infer,senses,kb=sensesKB)
        inferStep = infer.inference_steps[-1]
        
        return inferStep.output_dbs[-1]
        

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
        self.kbs = []
        inf_step = PRACInferenceStep(pracinference, self)
        planList = self.getPlanList()
        
        for db in dbs:
            db_ = db.duplicate()
            for q in db.query('action_core(?w,?ac)'):
                running = True
                #This list is used to avoid an infinite loop during the achieved by inference.
                #To avoid this infinite loop, the list contains the pracmlns which were inferenced during the process.
                #Every pracmln should be used only once during the process because the evidence for the inference will always remain the same.
                #So if the pracmln hadnt inferenced a plan in the first time, it will never do it.
                inferencedAchievedByList = []
                wordnet = WordNet(concepts=None)
                actionword = q['?w']
                actioncore = q['?ac']

                if kb is None:
                    print 'Loading Markov Logic Network: %s' % colorize(actioncore, (None, 'white', True), True)
                    if os.path.isfile(os.path.join(achievedByModulePath,'bin',actioncore+".pracmln")):
                        useKB = self.load_prac_kb(actioncore)
                    else:
                        running = False
                        inf_step.output_dbs.append(db_)
                        print actioncore + ".pracmln does not exist."
                else:
                    useKB = kb
                
                while running :
                    concepts = useKB.query_mln.domains.get('concept', [])
                    
                    for q in db_.query("has_sense(?w,?s)"):
                        for concept in concepts:
                            sim = wordnet.path_similarity(q["?s"], concept)
                            db_.addGroundAtom('is_a(%s,%s)' % (q["?s"], concept),sim)
                    #Inference achieved_by predicate        
                    self.kbs.append(useKB)  
                    params.update(useKB.query_params)
                    result_db = list(useKB.infer(db_))
                    
                    for r_db in result_db:
                        countOfGeneratorItems = 0
                        for q in r_db.query('achieved_by('+actionword+',?nac)'):
                            countOfGeneratorItems += 1
                            actioncore = q['?nac']
                            if actioncore in planList:
                                running = False
                                for atom, truth in sorted(db_.evidence.iteritems()):
                                    if 'is_a' in atom : continue
                                    r_db.addGroundAtom(atom,truth)
                                inf_step.output_dbs.append(r_db)
                                print actionword + " achieved by: " + actioncore
                                
                            elif actioncore in inferencedAchievedByList or not os.path.isfile(os.path.join(achievedByModulePath,'bin',actioncore+".pracmln")):
                                running = False
                                inf_step.output_dbs.append(db.duplicate())
                                if actioncore in inferencedAchievedByList:
                                    print "Could not inference a correct plan."
                                else:
                                    print actioncore + ".pracmln does not exist."
                                
                            else:
                                inferencedAchievedByList.append(actioncore)
                                resultSensesDB = self.updateActionRoles(db_,actionword,actioncore)
                                
                                #Prepare evidence db for the next inference of the achieved_by predicate
                                useKB = self.load_prac_kb(actioncore)
                                db_temp = Database(useKB.query_mln)
                                
                                for atom, truth in sorted(db_.evidence.iteritems()):
                                    if 'action_core' in atom or 'is_a' in atom or 'action_role' in atom: continue
                                    db_temp.addGroundAtom(atom,truth)
                                
                                for atom, truth in sorted(resultSensesDB.evidence.iteritems()):
                                    if 'action_role' in atom:
                                        db_temp.addGroundAtom(atom,truth)
                                        
                                db_temp.addGroundAtom('action_core('+actionword+","+actioncore+")")
                                db_ = db_temp
                                
                        if countOfGeneratorItems == 0:
                            running = False
                            inf_step.output_dbs.append(db.duplicate())
                            print "Could not inference a correct plan." 
            return inf_step
    
