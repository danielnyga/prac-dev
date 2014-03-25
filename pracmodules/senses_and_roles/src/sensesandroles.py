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
        
        inf_step = PRACInferenceStep(pracinference, self)
        for db in dbs:
            db_ = db.duplicate()
            for q in db.query('action_core(?w, ?ac)'):
                actioncore = q['?ac']
                if kb is None:
                    useKB = self.load_pracmt(actioncore)
                else:
                    useKB = kb
            roles = useKB.query_mln.domains.get('role', [])
            specified_roles = []
            for q in db.query('action_role(?w, ?r)'):
                specified_roles.append(q['?r'])
            unknown_roles = set(roles).difference(specified_roles)
            for i, role in enumerate(unknown_roles):
                if role == 'null': continue
                db_.addGroundAtom('action_role(skolem-%s, %s)' % (role, role))
            db_.write(sys.stdout, color=True)
#                 result = list(useKB.infer(db))
#             inf_step.output_dbs.extend(result_db)
        return inf_step
        
        
        
    
    
    
