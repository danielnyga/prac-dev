# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012 by Daniel Nyga (nyga@cs.tum.edu)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# 'Software'), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
import os
import yaml

import prac
from prac.core.base import PRACModule, PRACPIPE, PRACDatabase
from prac.core.inference import PRACInferenceStep
from prac.pracutils.utils import prac_heading
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize
from pracmln import praclog
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png
from pracmln.mln.errors import NoConstraintsError


logger = praclog.logger(__name__, praclog.INFO)
rolesTransformationModulePath = os.path.join(prac.locations.home, 'pracmodules', 'roles_transformation')
planListFilePath = os.path.join(rolesTransformationModulePath, 'plan_list.yaml')


class RolesTransformation(PRACModule):
    '''
    PRACModule used refine action roles after a preceding action core
    refinement.
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

        # ======================================================================
        # Initialization
        # ======================================================================

        logger.debug('inference on {}'.format(self.name))

        if self.prac.verbose > 0:
            print prac_heading('Update roles based on Action Core Refinement')

        inf_step = PRACInferenceStep(pracinference, self)
        planlist = self.getPlanList()
        dbs = pracinference.inference_steps[-1].output_dbs

        pngs = {}
        for i, db in enumerate(dbs):

            # ==================================================================
            # Preprocessing
            # ==================================================================

            skip_db = False
            for q in db.query('action_core(?w,?ac)'):
                actioncore = q['?ac']

                if actioncore in planlist:
                    skip_db = True

            if skip_db:
                inf_step.output_dbs.append(db)
                continue

            for q in db.query('achieved_by(?w,?ac)'):
                actioncore = q['?ac']
                logger.debug('Action core: {}'.format(actioncore))

                if params.get('project', None) is None:
                    logger.debug('Loading Project: {}.pracmln'.format(colorize(actioncore, (None, 'cyan', True), True)))
                    projectpath = os.path.join(self.module_path, '{}Transformation.pracmln'.format(actioncore))
                    project = MLNProject.open(projectpath)
                else:
                    logger.debug(colorize('Loading Project from params', (None, 'cyan', True), True))
                    projectpath = os.path.join(params.get('projectpath', None) or self.module_path,
                                               params.get('project').name)
                    project = params.get('project')

                mlntext = project.mlns.get(project.queryconf['mln'], None)
                mln = parse_mln(mlntext, searchpaths=[self.module_path],
                                projectpath=projectpath,
                                logic=project.queryconf.get('logic', 'FirstOrderLogic'),
                                grammar=project.queryconf.get('grammar', 'PRACGrammar'))

                result_db = None

                try:

                    # ==========================================================
                    # Inference
                    # ==========================================================

                    infer = self.mlnquery(config=project.queryconf, db=db,
                                          verbose=self.prac.verbose > 2,
                                          mln=mln)
                    result_db = infer.resultdb

                    if self.prac.verbose == 2:
                        print
                        print prac_heading('INFERENCE RESULTS')
                        print
                        infer.write()

                except NoConstraintsError:
                    result_db = db

                # ==============================================================
                # Postprocessing
                # ==============================================================

                r_db = PRACDatabase(self.prac)
                roles = self.prac.actioncores[actioncore].roles
                for atom, truth in sorted(result_db.evidence.iteritems()):
                    if any(r in atom for r in roles):
                        (_, predname, args) = self.prac.mln.logic.parse_literal(atom)
                        if args[-1] == actioncore:
                            r_db << (atom, truth)

                unified_db = db.union(r_db, mln=self.prac.mln)

                if actioncore not in planlist:
                    r_db_ = PRACDatabase(self.prac)
                    actionverb = ''

                    # It will be assumed that there is only one true action_
                    # core predicate per database
                    for q1 in unified_db.query('action_core(?w,?ac)'):
                        actionverb = q1['?w']

                    for atom, truth in sorted(unified_db.evidence.iteritems()):
                        if 'action_core' in atom: continue
                        r_db_ << (atom, truth)
                    r_db_ << ('action_core({},{})'.format(actionverb, actioncore))
                    inf_step.output_dbs.append(r_db_)
                else:
                    self.isLastActionCoreAPlan = True
                    inf_step.output_dbs.append(unified_db)

            pngs['RolesTransformation - ' + str(i)] = get_cond_prob_png(project.queryconf.get('queries', ''), dbs,
                                                                        filename=self.name)
            inf_step.png = pngs
            inf_step.applied_settings = project.queryconf.config
        return inf_step
