# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2016 by Sebastian Koralewski (seba@informatik.uni-bremen.de)
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
from pracmln.mln.base import parse_mln
from pracmln.mln.methods import LearningMethods
from prac.core.inference import PRACInferenceStep
# mapping from PennTreebank POS tags to NLTK POS Tags
from pracmln import Database, MLN, MLNQuery
from pracmln.mln.util import colorize, out, stop
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png
from wx import PreDatePickerCtrl


log = logger(__name__)


class ControlStructureMerger(PRACModule):
    
    def initialize(self):
        pass

    @PRACPIPE
    def __call__(self, pracinference, **params):
        log.debug('inference on %s' % self.name)
        
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: Control Structure Merger |', (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)
        print
        #print colorize('Inferring most probable ACTION CORE', (None, 'white', True), True)

        dbs = pracinference.inference_steps[-1].output_dbs

        
        inf_step = PRACInferenceStep(pracinference, self)
        
        db_map = {}
        for db in dbs:
            cs_names = sorted(db.domains.get('cs_name', []))
            if cs_names:
                cs_name = cs_names[-1]
                if cs_name in db_map.keys():
                    db_map[cs_name].append(db.copy())
                else:
                    db_map[cs_name] = [db.copy()]
            
            else:
                inf_step.output_dbs.append(db)
        
        for _ , database_list in db_map.iteritems():
            
            merged_db = database_list[0]
            merged_mln = merged_db.mln
            #Merge MLN predicate definitions
            
            for database in database_list[1:]:
                merged_mln.update_predicates(database.mln)
            
            inf_step.output_dbs.append(merged_db.union(database_list))
            
        return inf_step