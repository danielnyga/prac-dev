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
rolesTransformationModulePath = os.path.join(PRAC_HOME, 'pracmodules', 'roles_transformation')
planListFilePath = os.path.join(rolesTransformationModulePath,'plan_list.yaml')

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
            dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
            dbs = kb.dbs

        inf_step = PRACInferenceStep(pracinference, self)
        planlist = self.getPlanList()
        
        for db in dbs:

            for q in db.query('achieved_by(?w,?ac)'):
                actioncore = q['?ac']
                log.info(actioncore)

                if kb is None:
                    log.info('Loading Markov Logic Network: %s' % colorize(actioncore+'Transformation', (None, 'cyan', True), True))
                    kb = self.load_prac_kb(actioncore+'Transformation')

                result_db = list(kb.infer(db))[0]
                unified_db = db.union(result_db)

                if actioncore not in planlist:
                    # ONLY LEAVE POSITIVE ACTION CORE /UPDATE /TODO
                    r_db_ = Database(self.prac.mln)
                    actionverb = ""

                    #It will be assumed that there is only one true action_core predicate per database
                    for q1 in unified_db.query("action_core(?w,?ac)"):
                        actionverb = q1["?w"]

                    for atom, truth in sorted(unified_db.evidence.iteritems()):
                        if 'action_core' in atom: continue
                        r_db_ << (atom, truth)
                    r_db_ << ("action_core({},{})".format(actionverb, actioncore))
                    inf_step.output_dbs.append(r_db_)
                else:
                    self.isLastActionCoreAPlan = True
                    inf_step.output_dbs.append(unified_db)


        log.info("Generating png")
        if kb is not None:
            png, ratio = kb.get_cond_prob_png(filename=self.name)
            inf_step.png = (png, ratio)
            inf_step.applied_kb = kb.filename                
        return inf_step