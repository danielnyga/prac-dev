# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2013 by Daniel Nyga (nyga@cs.tum.edu)
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

from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from prac.pracutils.utils import prac_heading
from pracmln import MLNQuery
from pracmln import praclog
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


logger = praclog.logger(__name__, praclog.INFO)


class ControlStructureIdentification(PRACModule):
    

    def initialize(self):
        pass

    @PRACPIPE
    def __call__(self, pracinference, **params):

        # ======================================================================
        # Initialization
        # ======================================================================

        logger.debug('inference on {}'.format(self.name))

        if self.prac.verbose > 0:
            print prac_heading('Recognizing Control Structures')

        if params.get('project', None) is None:
            # load default project
            projectpath = self.project_path
            ac_project = MLNProject.open(projectpath)
        else:
            logger.info(colorize('Loading Project from params', (None, 'cyan', True), True))
            projectpath = os.path.join(params.get('projectpath', None) or self.module_path, params.get('project').name)
            ac_project = params.get('project')

        dbs = pracinference.inference_steps[-1].output_dbs
        
        mlntext = ac_project.mlns.get(ac_project.queryconf['mln'], None)
        mln = parse_mln(mlntext, searchpaths=[self.module_path], projectpath=projectpath, logic=ac_project.queryconf.get('logic', 'FirstOrderLogic'), grammar=ac_project.queryconf.get('grammar', 'PRACGrammar'))
        inf_step = PRACInferenceStep(pracinference, self)

        pngs = {}
        for i, db in enumerate(dbs):
            db_ = db.copy()

            # ======================================================================
            # Inference
            # ======================================================================

            infer = MLNQuery(config=ac_project.queryconf, db=db, mln=mln).run()
            result_db = infer.resultdb

            if self.prac.verbose == 2:
                print
                print prac_heading('INFERENCE RESULTS')
                print
                infer.write()

            # ==========================================================
            # Postprocessing
            # ==========================================================

            for q in result_db.query('event(?w,?ac)'):
                db_ << 'event({},{})'.format(q['?w'],q['?ac'])
            for q in result_db.query('condition(?w)'):
                db_ << 'condition({})'.format(q['?w'])
                
            inf_step.output_dbs.append(db_)
            pngs['CS' + str(i)] = get_cond_prob_png(ac_project.queryconf.get('queries', ''), dbs, filename=self.name)
            inf_step.png = pngs

        inf_step.applied_settings = ac_project.queryconf.config
    
        return inf_step
    