# PROBABILISTIC ROBOT ACTION CORES
#
# (C) 2016 by Mareike Picklum (mareikep@cs.uni-bremen.de)
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
from pracmln import praclog, MLNQuery
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


log = praclog.logger(__name__)


class CorefResolution(PRACModule):
    """
    Pracmodule used to perform coreference resolution.
    """


    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+==========================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RESOLVING COREFERENCES    ',
                       (None, 'green', True), True)
        print colorize('+==========================================+',
                       (None, 'green', True), True)

        if params.get('project', None) is None:
            # load default project
            projectpath = self.project_path
            ac_project = MLNProject.open(projectpath)
        else:
            log.info(
                colorize('Loading Project from params', (None, 'cyan', True),
                         True))
            projectpath = os.path.join(
                params.get('projectpath', None) or self.module_path,
                params.get('project').name)
            ac_project = params.get('project')

        dbs = pracinference.inference_steps[-1].output_dbs

        mlntext = ac_project.mlns.get(ac_project.queryconf['mln'], None)
        mln = parse_mln(mlntext,
                        searchpaths=[self.module_path],
                        projectpath=projectpath,
                        logic=ac_project.queryconf.get('logic',
                                                       'FirstOrderLogic'),
                        grammar=ac_project.queryconf.get('grammar',
                                                         'PRACGrammar'))

        inf_step = PRACInferenceStep(pracinference, self)

        infer = MLNQuery(config=ac_project.queryconf, db=tmp_union_db,
                         mln=mln).run()
        result_db = infer.resultdb

        inf_step.output_dbs.append(unified_db)

        png, ratio = get_cond_prob_png(ac_project.queryconf.get('queries', ''),
                                       dbs, filename=self.name)
        inf_step.png = (png, ratio)
        inf_step.applied_settings = ac_project.queryconf.config
        return inf_step






        # for olddb in dbs:
        #     olddb.write(bars=None)
        #
        # txt = ''
        # with open('/home/mareikep/Desktop/testmln.mln') as infile:
        #     txt = infile.read()
        # mln = parse_mln(text=txt)
        # mln.write(color=None)
        # return inf_step
