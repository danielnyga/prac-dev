# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2015 by Daniel Nyga (nyga@cs.tum.edu)
# (C) 2015 by Sebastian Koralewski (seba@informatik.uni-bremen.de)
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
from prac.core.base import PRACModule, PRACPIPE, DB_TRANSFORM
from prac.core.inference import PRACInferenceStep
from pracmln import Database, MLNQuery
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, out
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet


log = logger(__name__)
PRAC_HOME = os.environ['PRAC_HOME']
corpus_path_list = os.path.join(PRAC_HOME, 'corpus')

class ComplexAchievedBy(PRACModule):
    '''

    '''

    def initialize(self):
        pass

    def shutdown(self):
        pass

    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: PROCESSING COMPLEX ACHIEVED BY  ' , (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs
        prac = PRAC()
        prac.wordnet = WordNet(concepts=None)
        
        for olddb in dbs:
            for q in olddb.query('achieved_by(?w,?ac)'):
                actioncore = q['?ac']
                text_file = open(os.path.join(corpus_path_list,actioncore), 'r')
                sentences = text_file.readlines()

                for s in sentences:
                    sub_inference = PRACInference(prac, [s])

                    while sub_inference.next_module() != 'plan_generation' :
                        modulename = sub_inference.next_module()
                        module = prac.getModuleByName(modulename)
                        prac.run(sub_inference, module)

                    inf_step.output_dbs.extend(sub_inference.inference_steps[-1].output_dbs)
                
                

            
        return inf_step
    
