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
from prac.inference import PRACInferenceStep
from prac.core import PRACModule
import copy

class PlanGenerator(PRACModule):
    
    def __call__(self, pracinference, **params):
        print 'generating CRAM plan'
        infstep = PRACInferenceStep(pracinference, self)
        infstep.output_dbs = list(pracinference.inference_steps[-1].output_dbs)
        infstep.executable_plans = []
        for db in infstep.output_dbs:
            for query in ('achieved_by(?a1, ?a)', 'action_core(?w, ?a)'):
                for q in db.query(query):
                    actioncore = q['?a']
                    ac = self.prac.actioncores.get(actioncore)
                    if not ac.plan: continue
                    assignment = {}
                    role_assignments = [assignment]
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
                    for assignment in role_assignments:
                        infstep.executable_plans.append(ac.parameterize_plan(**assignment))
                    break
        print infstep.executable_plans
        return infstep