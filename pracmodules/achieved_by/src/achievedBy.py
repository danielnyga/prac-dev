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
                #running = True
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
                        useKB = self.load_pracmt(actioncore)
                    else:
                        inf_step.output_dbs.append(db_)
                        print actioncore + ".pracmln does not exist."
                else:
                    useKB = kb
                
                concepts = useKB.query_mln.domains.get('concept', [])
                
                for q in db_.query("has_sense(?w,?s)"):
                    for concept in concepts:
                        sim = wordnet.path_similarity(q["?s"], concept)
                        db_.addGroundAtom('is_a(%s,%s)' % (q["?s"], concept),sim)
                
                #Inference achieved_by predicate        
                #self.kbs.append(useKB)  
                #params.update(useKB.query_params)
                result_db = list(useKB.infer(db_))
                result_db_ = []
                
                #Removing achieved_by predicates with prob zero
                for r_db in result_db:
                    r_db_ = Database(useKB.query_mln)
                    for atom, truth in sorted(r_db.evidence.iteritems()):
                        if 'achieved_by' in atom and truth == 0: continue
                        r_db_.addGroundAtom(atom,truth)
                    result_db_.append(r_db_)
                
                result_db = result_db_
                    
                for r_db in result_db:
                    for q in r_db.query('achieved_by('+actionword+',?nac)'):
                        achievedByAc = q['?nac']
                        removingPredicateList = ['is_a']
                        r_db_ = Database(r_db.mln)
                        
                        if achievedByAc not in planList:
                            removingPredicateList.append("action_core")
                            
                        for atom, truth in sorted(r_db.evidence.iteritems()):
                            if atom.split("(")[0] in removingPredicateList: continue
                            r_db_.addGroundAtom(atom,truth)
                        
                        if achievedByAc not in planList:
                            r_db_.addGroundAtom('action_core('+actionword+","+achievedByAc+")")
                        inf_step.output_dbs.append(r_db_)
                        print actionword + " achieved by: " + actioncore
                            
                return inf_step
    
