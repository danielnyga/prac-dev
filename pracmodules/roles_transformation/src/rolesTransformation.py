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
from prac.pracutils.ActioncoreDescriptionHandler import \
    ActioncoreDescriptionHandler
from pracmln import Database, MLNQuery
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, out
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


PRAC_HOME = os.environ['PRAC_HOME']
rolesTransformationModulePath = os.path.join(PRAC_HOME, 'pracmodules', 'roles_transformation')
planListFilePath = os.path.join(rolesTransformationModulePath,'plan_list.yaml')


log = logger(__name__)


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
    
    def getComplexList(self):
        planListFile = open(planListFilePath, 'r')
        yamlData = yaml.load(planListFile)
        
        return yamlData['complexList']
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+===================================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: Update roles based on achieved_by |' , (None, 'green', True), True)
        print colorize('+===================================================+', (None, 'green', True), True)
        
        inf_step = PRACInferenceStep(pracinference, self)
        planlist = self.getPlanList()
        dbs = pracinference.inference_steps[-1].output_dbs

        for db in dbs:

            for q in db.query('achieved_by(?w,?ac)'):
                actioncore = q['?ac']
                log.info('Action core: {}'.format(actioncore))

                if params.get('project', None) is None:
                    log.info('Loading Project: %s.pracmln' % colorize(actioncore, (None, 'cyan', True), True))
                    projectpath = os.path.join(self.module_path, '{}Transformation.pracmln'.format(actioncore))
                    project = MLNProject.open(projectpath)
                else:
                    log.info(colorize('Loading Project from params', (None, 'cyan', True), True))
                    projectpath = os.path.join(params.get('projectpath', None) or self.module_path, params.get('project').name)
                    project = params.get('project')

                mlntext = project.mlns.get(project.queryconf['mln'], None)
                mln = parse_mln(mlntext, searchpaths=[self.module_path], projectpath=projectpath, logic=project.queryconf.get('logic', 'FirstOrderLogic'), grammar=project.queryconf.get('grammar', 'PRACGrammar'))

                db.write(bars=False)

                infer = MLNQuery(config=project.queryconf, db=db, mln=mln).run()
                result_db = infer.resultdb

                r_db = Database(self.prac.mln)
                roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
                for atom, truth in sorted(result_db.evidence.iteritems()):
                    if any(r in atom for r in roles):
                        (_, predname, args) = self.prac.mln.logic.parse_literal(atom)
                        if args[-1] == actioncore:
                            r_db << (atom, truth)

                unified_db = db.union(r_db, mln=self.prac.mln)

                if actioncore not in planlist:
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


            png, ratio = get_cond_prob_png(project.queryconf.get('queries', ''), dbs, filename=self.name)
            inf_step.png = (png, ratio)
            inf_step.applied_settings = project.queryconf.config
        return inf_step