# PROBABILISTIC ROBOT ACTION CORES - INFERENCE
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

from prac.pracutils import StopWatch
from graphviz.dot import Digraph
from prac.pracutils.pracgraphviz import render_gv
from pracmln import Database


class PRACInferenceStep(object):
    """
    Wrapper class encapsulating a single inference step in the PRAC
    pipeline. It consists of n input databases and m output databases.
    """


    def __init__(self, pracinfer, module, input_dbs=None):
        '''
        Initializes the inference step.
        - pracinfer:    reference to the PRACInference object.
        - module:       reference to the PRACModule performing this inference step.
        - input_dbs:    list of databases taken as inputs.
        '''
        if input_dbs is None:
            self.input_dbs = []
        else:
            self.input_dbs = input_dbs
        self.module = module
        self.prac = pracinfer.prac
        self.pracinference = pracinfer
        self.output_dbs = []
        self.watch = StopWatch()


class PRACInference(object):
    """
    Represents an inference chain in PRAC:
    - prac:            reference to the PRAC instance.
    - instructions:    list of natural-language sentences subject to inference.
    """


    def __init__(self, prac, instructions):
        self.prac = prac
        prac.uninitAllModules()
        self.instructions = instructions
        self.inference_steps = []
        self.watch = StopWatch()


    def next_module(self):
        if not self.inference_steps:
            return 'nl_parsing'
        previous_module = self.inference_steps[-1].module.name

        if previous_module == 'nl_parsing':
            return 'ac_recognition'
        elif previous_module == 'ac_recognition':
            return 'prop_extraction'
        elif previous_module == 'prop_extraction':
            return 'senses_and_roles'
        elif previous_module == 'senses_and_roles':
            return 'coref_resolution'
        elif previous_module == 'coref_resolution':
            for outdb in self.inference_steps[-1].output_dbs:
                
                if self.is_task_missing_roles(outdb):
                    return 'role_look_up'
                
                for r in outdb.query('action_core(?w, ?a)'):
                    actioncore = r['?a']
                    mod = self.prac.getModuleByName('roles_transformation')
                    plans = mod.getPlanList()
                    if actioncore not in plans: return 'achieved_by'

            return 'plan_generation'
        elif previous_module == 'role_look_up':
            for outdb in self.inference_steps[-1].output_dbs:
                for r in outdb.query('action_core(?w, ?a)'):
                    actioncore = r['?a']
                    mod = self.prac.getModuleByName('roles_transformation')
                    plans = mod.getPlanList()
                    if actioncore not in plans: return 'achieved_by'

                return 'plan_generation'

        elif previous_module == 'achieved_by':
            # TODO ADD complex achieved by support
            for outdb in self.inference_steps[-1].output_dbs:
                for r in outdb.query('achieved_by(?w, ?a)'):
                    actioncore = r['?a']
                    if actioncore == 'Complex':
                        return 'complex_achieved_by'
                    else:
                        return 'roles_transformation'
        elif previous_module == 'roles_transformation':
            for outdb in self.inference_steps[-1].output_dbs:
                for r in outdb.query('achieved_by(?w,?a)'):
                    actioncore = r['?a']
                    mod = self.prac.getModuleByName('roles_transformation')
                    plans = mod.getPlanList()
                    if actioncore not in plans:
                        return 'achieved_by'
            return 'plan_generation'
        elif previous_module == 'complex_achieved_by':
            return 'plan_generation'
        elif previous_module == 'plan_generation':
            return None

        return None


    def finalgraph(self, filename=None):
        finaldb = Database(self.prac.mln)
        for step in self.inference_steps:
            for db in step.output_dbs:
                for atom, truth in db.evidence.iteritems():
                    if truth == 0: continue
                    _, predname, args = self.prac.mln.logic.parseLiteral(atom)
                    if predname in self.prac.roles.union(['has_sense', 'action_core', 'achieved_by']):
                        finaldb << atom
                    #         finaldb.write(sys.stdout, color=True)
        g = Digraph(format='svg', engine='dot')
        g.attr('node', shape='box', style='filled')
        for res in finaldb.query('action_core(?w, ?a) ^ has_sense(?w, ?s)'):
            actioncore = res['?a']
            sense = res['?s']
            predname = 'action_core'
            g.node(actioncore, fillcolor='#bee280')
            g.node(sense)
            g.edge(actioncore, sense, label='is_a')
            roles = self.prac.actioncores[actioncore].roles
            for role in roles:
                for res in db.query('%s(?w, %s) ^ has_sense(?w, ?s)' % (
                role, actioncore)):
                    sense = res['?s']
                    g.node(sense)
                    g.edge(actioncore, sense, label=role)
        for res in finaldb.query('achieved_by(?a1, ?a2)'):
            a1 = res['?a1']
            a2 = res['?a2']
            g.node(a1, fillcolor='#bee280')
            g.node(a2, fillcolor='#bee280')
            g.edge(a1, a2, label='achieved_by')
            actioncore = a2
            roles = self.prac.actionroles[actioncore].roles
            for role in roles:
                for res in db.query('%s(?w, %s) ^ has_sense(?w, ?s)' % (
                role, actioncore)):
                    sense = res['?s']
                    g.node(sense)
                    g.edge(actioncore, sense, label=role)
        return render_gv(g, filename)


    def is_task_missing_roles(self, db):
        # Assuming there is only one action core
        for q in db.query('action_core(?w,?ac)'):
            actioncore = q['?ac']

            roles_senses_dict = {k: v for r in db.roles(actioncore) for k, v in r.items()}

            inferred_roles_set = set(roles_senses_dict.keys())

            # Determine missing roles: All_Action_Roles\Inferred_Roles
            actioncore_roles_list = self.prac.actioncores[actioncore].required_roles

            if not actioncore_roles_list:
                actioncore_roles_list = self.prac.actioncores[actioncore].roles
                
            missing_role_set = set(actioncore_roles_list).difference(inferred_roles_set)

            if missing_role_set:
                return True
            return False
        return False
