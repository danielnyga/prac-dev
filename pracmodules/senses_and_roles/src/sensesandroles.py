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

from prac.core import PRACModule, PRACPIPE, PRACKnowledgeBase
import logging
from mln import readMLNFromFile, readDBFromFile#, MLNParsingError
from mln.methods import LearningMethods
import sys
from wcsp.converter import WCSPConverter
from mln.database import Database
import os
from prac.inference import PRACInferenceStep
from mln.util import mergeDomains


class SensesAndRoles(PRACModule):
    '''
    
    '''
    
    def initialize(self):
        pass
    
    def shutdown(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        kb = params.get('kb', None)
        if kb is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
            dbs = kb.dbs
        self.kbs = []
        inf_step = PRACInferenceStep(pracinference, self)
        for db in dbs:
            db_ = db.duplicate()
            db_.write(sys.stdout, color=True)
            for q in db.query('action_core(?w, ?ac)'):
                actioncore = q['?ac']
                if actioncore == 'null': continue
                if kb is None:
                    useKB = self.load_pracmt(actioncore)
                else:
                    useKB = kb
                self.kbs.append(useKB)
                params.update(useKB.query_params)
                if 'missing' in params:
                    roles = useKB.query_mln.domains.get('role', [])
                    log.info('roles: %s' % roles)
                    specified_roles = []
                    for q in db.query('action_role(?w, ?r)'):
                        specified_roles.append(q['?r'])
                    unknown_roles = set(roles).difference(set(specified_roles))
                    log.info('unknown roles: %s' % unknown_roles)
                    for i, role in enumerate(unknown_roles):
                        if role == 'null': continue
                        log.info('adding %s' % ('action_role(skolem-%s, %s)' % (role, role)))
                        db_.addGroundAtom('action_role(skolem-%s, %s)' % (role, role))
                concepts = useKB.query_mln.domains['concept']#mergeDomains(, self.merge_all_domains(pracinference))['concept']
                log.info('adding senses. concepts=%s' % concepts)
                db_ = self.prac.getModuleByName('wn_senses').add_senses_and_similiarities_for_concepts(db_, concepts)
                result_db = list(useKB.infer(db_))
                db_ = db_.union(None, *result_db)
    #             db_.write(sys.stdout, color=True)
                inf_step.output_dbs.append(db_)
        return inf_step
        
        
        
    
    
    
