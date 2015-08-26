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
import os
import yaml

from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from pracmln import Database
from pracmln.mln.util import colorize
from pracmln.praclog import logger

PRAC_HOME = os.environ['PRAC_HOME']
achievedByModulePath = os.path.join(PRAC_HOME, 'pracmodules', 'roles_transformation')
planListFilePath = os.path.join(achievedByModulePath,'plan_list.yaml')



class RolesTransformation(PRACModule):
    '''
    
    '''
        
    def initialize(self):
        self.isLastActionCoreAPlan = False
        
    def shutdown(self):
        pass
    
    def getPlanList(self):
        planListFile = open(planListFilePath, 'r')
        yamlData = yaml.load(planListFile)
        
        return yamlData['planList']
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logger(self.name)
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: Update roles based on achieved_by' , (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)
        
        kb = params.get('kb', None)
        if kb is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
            dbs = kb.dbs
        
        inf_step = PRACInferenceStep(pracinference, self)
        planList = self.getPlanList()
        
        for db in dbs:
            db_ = db.duplicate()
            for q in db.query('achieved_by(?w,?ac)'):
                actioncore = q['?ac']

                if kb is None:
                    print 'Loading Markov Logic Network: %s' % colorize(actioncore+'Transformation', (None, 'white', True), True)
                    useKB = self.load_pracmt(actioncore+'Transformation')
                else:
                    useKB = kb
                
                result_db = list(useKB.infer(db_))
                if actioncore not in planList:
                    for r_db in result_db:
                        r_db_ = Database(r_db.mln)
                        actionverb = ""
                        
                        #It will be assumed that there is only one true action_core predicate per database 
                        for q1 in db.query("action_core(?w,?ac)"):
                            actionverb = q1["?w"]
                            
                        for atom, truth in sorted(r_db.evidence.iteritems()):
                            if 'action_core' in atom: continue
                            r_db_ << (atom,truth)
                        r_db_ << ("action_core("+actionverb+","+actioncore+")")
                        inf_step.output_dbs.append(r_db_)
                else:
                    self.isLastActionCoreAPlan = True
                    inf_step.output_dbs.extend(result_db)

                kb = useKB

        print 'generatinig png'
        if kb is not None:
            print 'blubbel'
            png, ratio = kb.get_cond_prob_png(filename=self.name)
            inf_step.png = (png, ratio)
            inf_step.applied_kb = kb.filename                
        return inf_step