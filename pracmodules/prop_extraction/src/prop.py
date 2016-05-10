# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2014 by Mareike Picklum (mareikep@cs.uni-bremen.de)
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
import sys
from prac.core.base import PRACModule, PRACPIPE, PRAC
from prac.core.inference import PRACInferenceStep, PRACInference
from prac.core.wordnet import WordNet
from pracmln import MLNQuery
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


log = logger(__name__)


class PropExtraction(PRACModule):
    def initialize(self):
        pass


    @PRACPIPE
    def __call__(self, pracinference, **params):
        log.info('Running {}'.format(self.name))

        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC PROPERTY EXTRACTION                    |',
                       (None, 'green', True), True)
        print colorize('+=============================================+',
                       (None, 'green', True), True)
        print
        print colorize(
            'Inferring most probable ANNOTATION + simultaneous WORD SENSE '
            'DISMABIGUATION...',
            (None, 'white', True), True)

        if params.get('project', None) is None:
            # load default project
            projectpath = self.project_path
            project = MLNProject.open(projectpath)
        else:
            log.info(
                colorize('Loading Project from params', (None, 'cyan', True),
                         True))
            projectpath = os.path.join(
                params.get('projectpath', None) or self.module_path,
                params.get('project').name)
            project = params.get('project')

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs

        mlntext = project.mlns.get(project.queryconf['mln'], None)
        mln = parse_mln(mlntext, searchpaths=[self.module_path],
                        projectpath=projectpath,
                        logic=project.queryconf.get('logic', 'FuzzyLogic'),
                        grammar=project.queryconf.get('grammar',
                                                      'PRACGrammar'))
        wordnet_module = self.prac.getModuleByName('wn_senses')

        # process databases
        for db in dbs:
            db = wordnet_module.add_sims(db, mln)

            db.write(bars=False)

            # infer and update output
            infer = MLNQuery(config=project.queryconf, db=db, mln=mln).run()
            result_db = infer.resultdb

            unified_db = db.union(result_db)

            inf_step.output_dbs.append(unified_db)

        png, ratio = get_cond_prob_png(project.queryconf.get('queries', ''),
                                       dbs, filename=self.name)
        inf_step.png = (png, ratio)
        inf_step.applied_settings = project.queryconf.config
        return inf_step


if __name__ == '__main__':

    if len(sys.argv) < 2:
        sys.exit(-1)
    sentences = [sys.argv[1]]
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)

    inference = PRACInference(prac, sentences)

    if len(inference.instructions) > 0:
        parser = prac.getModuleByName('nl_parsing')
        prac.run(inference, parser)

    modules = ['ac_recognition', 'senses_and_roles', 'prop_extraction']
    for mname in modules:
        module = prac.getModuleByName(mname)
        prac.run(inference, module)
