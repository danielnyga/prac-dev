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
import os
import itertools
from prac.core.base import PRACModule
from prac.core.inference import PRACInferenceStep
from prac.pracutils.utils import prac_heading
from pracmln import MLN
from pracmln import praclog
from pracmln.mln.util import colorize


logger = praclog.logger(__name__, praclog.INFO)


class PlanGenerator(PRACModule):
    '''
    PRACModule used to generate a parameterized plan from information inferred
    by previous modules.
    '''

    def __call__(self, node, **params):

        # ======================================================================
        # Initialization
        # ======================================================================

        logger.debug('Running {}'.format(self.name))

        if self.prac.verbose > 0:
            print prac_heading('Generating CRAM Plan(s)')

        dbs = node.outdbs
        infstep = PRACInferenceStep(node, self)
        infstep.inferred_roles = {}
        infstep.executable_plans = []
        properties = [p.name for p in MLN.load(os.path.join(self.module_path, '..',
                                                            'prop_extraction', 'mln',
                                                            'predicates.mln')).predicates]

        for db in dbs:

            # ==================================================================
            # Preprocessing
            # ==================================================================

            for query in ['achieved_by(?ac1, ?ac)', 'action_core(?w, ?ac)']:
                for q in db.query(query):

                    actioncore = q['?ac']
                    ac = self.prac.actioncores.get(actioncore)
                    if not ac.plan:
                        continue

                    roleass = {}
                    role_assignments = []
                    for role in ac.roles:
                        if role == 'action_verb': continue
                        roleass[role] = list(db.query('{0}(?w,{1}) ^ has_sense(?w, ?s)'.format(role, actioncore)))

                    emptyroles = []

                    # remove empty roles
                    for r in roleass.keys():
                        if not roleass[r]:
                            emptyroles.append(r)
                            del roleass[r]

                    # itertools.product will return [] if roleass contains
                    # roles with empty lists, therefore we removed them
                    # before and re-add them later
                    combis = list((dict(itertools.izip(roleass, x)) for x in itertools.product(*roleass.itervalues())))

                    for c in combis:

                        # re-add empty roles
                        for e in emptyroles:
                            c[e] = {}

                        ass = {}
                        for role in c:
                            word = c[role].get('?w', 'Unknown')
                            sense = c[role].get('?s', 'Unknown')
                            ass[role] = sense

                            # add properties to the objects of the according roles
                            for prop in properties:
                                for rq2 in db.query('{0}({1},?w2) ^ has_sense(?w2, ?s2)'.format(prop, word)):
                                    if '{}_props'.format(role) in ass:
                                        ass['{}_props'.format(role)] += ' ({} {})'.format(prop, rq2['?s2'])
                                    else:
                                        ass['{}_props'.format(role)] = ' ({} {})'.format(prop, rq2['?s2'])

                            if '{}_props'.format(role) not in ass:
                                ass['{}_props'.format(role)] = ''

                            # Handle missing plan parameters
                            if role not in ass.keys():
                                ass[role] = "Unknown"

                        role_assignments.append(ass)

                    # ==========================================================
                    # Plan Parameterization
                    # ==========================================================

                    for assignment in role_assignments:
                        infstep.inferred_roles[ac.name] = assignment
                        node.plan = ac.parameterize_plan(**assignment)

                    if self.prac.verbose:
                        print
                        print prac_heading('PLAN GENERATION RESULTS')
                        print colorize('actioncore:', (None, 'white', True), True), colorize(ac.name, (None, 'cyan', True), True)
                        print colorize('assignments:', (None, 'white', True), True)
                        for assignment in role_assignments:
                            for x in assignment:
                                print '\t{}: {}'.format(colorize(x, (None, 'white', True), True), colorize(assignment[x], (None, 'cyan', True), True))
                    break
        return []
