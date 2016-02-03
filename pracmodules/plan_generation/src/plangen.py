# 
#
# (C) 2011-2015 by Daniel Nyga (nyga@cs.uni-bremen.de)
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
import copy
from prac.core.base import PRACModule
from prac.core.inference import PRACInferenceStep
from pracmln.mln.util import out
from pracmln.praclog import logger

log = logger(__name__)


class PlanGenerator(PRACModule):

    def __call__(self, pracinference, **params):
        out('generating CRAM plan')
        infstep = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs
        infstep.output_dbs = dbs
        infstep.executable_plans = []
        for db in dbs:

            for query in ('achieved_by(?ac1, ?ac)', 'action_core(?w, ?ac)'):
                for q in db.query(query):
                    actioncore = q['?ac']
                    ac = self.prac.actioncores.get(actioncore)
                    if not ac.plan:
                        continue
                    assignment = {}
                    role_assignments = [assignment]
                    out('roles: ')
                    out(ac.roles)
                    for role in ac.roles:
                        for i, rq in enumerate(db.query('{0}(?w,{1}) ^ has_sense(?w, ?s)'.format(role, actioncore))):
                            if i > 0:
                                for ass in list(role_assignments):
                                    new_ass = copy.copy(ass)
                                    role_assignments.append(new_ass)
                                    new_ass[role] = rq['?s']
                            else:
                                for ass in role_assignments:
                                    ass[role] = rq['?s']
                        # Handle missing plan parameters
                        if role not in ass.keys():
                            ass[role] = "Unknown"
                    for assignment in role_assignments:
                        infstep.executable_plans.append(
                            ac.parameterize_plan(**assignment))
                    break
        return infstep
