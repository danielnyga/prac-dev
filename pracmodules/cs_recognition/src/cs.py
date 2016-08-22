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
from pracmln.mln.base import parse_mln
from pracmln.mln.methods import LearningMethods
from prac.core.inference import PRACInferenceStep
# mapping from PennTreebank POS tags to NLTK POS Tags
from pracmln import Database, MLN, MLNQuery
from pracmln.mln.util import colorize, out, stop
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png
from pracmln.mln.errors import NoConstraintsError
from pracmln.mln.base import Predicate

log = logger(__name__)


class ControlStructureIdentification(PRACModule):
    

    def initialize(self):
        pass

    @PRACPIPE
    def __call__(self, pracinference, **params):
        log.debug('inference on %s' % self.name)
        
        print colorize('+==============================================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RECOGNIZING CONTROL STRUCTURE IDENTIFICATION |', (None, 'green', True), True)
        print colorize('+==============================================================+', (None, 'green', True), True)
        print
        #print colorize('Inferring most probable ACTION CORE', (None, 'white', True), True)

        if params.get('project', None) is None:
            # load default project
            projectpath = self.project_path
            ac_project = MLNProject.open(projectpath)
        else:
            log.info(colorize('Loading Project from params', (None, 'cyan', True), True))
            projectpath = os.path.join(params.get('projectpath', None) or self.module_path, params.get('project').name)
            ac_project = params.get('project')

        dbs = pracinference.inference_steps[-1].output_dbs
        
        mlntext = ac_project.mlns.get(ac_project.queryconf['mln'], None)
        mln = parse_mln(mlntext, searchpaths=[self.module_path], projectpath=projectpath, logic=ac_project.queryconf.get('logic', 'FirstOrderLogic'), grammar=ac_project.queryconf.get('grammar', 'PRACGrammar'))
        inf_step = PRACInferenceStep(pracinference, self)

        pngs = {}
        num_of_control_structures = 1
        
        for i, db in enumerate(dbs):
            db_ = db.copy()
            final_result = db.copy()
            db_  << "cs_name(CS-{})".format(str(num_of_control_structures))
            # result_db = list(kb.infer(tmp_union_db))[0]
            
            try:
                infer = MLNQuery(config=ac_project.queryconf, db=db_, mln=mln).run()
                result = infer.resultdb
                
                #Determine if control structure is "if" or "while"
                #Assuming there is only one cs in database
                condition_type = ""
                for q1 in result.query('condition_type(?w)'):
                    word_const = ('-'.join(q1['?w'].split('-')[:-1])).lower().strip()
                    if word_const == "else":
                        for q2 in db_.query('has_pos({},?p)'.format(q1['?w'])):
                            if q2['?p'] == 'UH':
                                condition_type = "else"
                            else:
                                condition_type = "else_if"
                    else:
                        condition_type = word_const
                
                if condition_type.startswith("else"):
                    final_result.mln.declare_predicate(Predicate(condition_type,['cs_name','cs_name']))
                    final_result << "{}(CS-{},CS-{})".format(condition_type,
                                                            str(num_of_control_structures-1),
                                                            str(num_of_control_structures))
                
                for query_predicate in ['condition','event']:
                    for q in result.query('{}(?w,?cs)'.format(query_predicate)):
                        updated_predicate = "{}_{}".format(condition_type,query_predicate)
                        updated_atom = "{}({},{})".format(updated_predicate,q['?w'],q['?cs'])
                        final_result.mln.declare_predicate(Predicate(updated_predicate,['actioncore','cs_name']))
                        final_result << updated_atom
                
                inf_step.output_dbs.append(final_result)
                
                #Database contains control structure since the MLN can be grounded
                num_of_control_structures += 1
            except NoConstraintsError:
                #Database does not contain control structure since the MLN cannot be grounded
                inf_step.output_dbs.append(db)
                
            pngs['CS' + str(i)] = get_cond_prob_png(ac_project.queryconf.get('queries', ''), dbs, filename=self.name)
            inf_step.png = pngs

        inf_step.applied_settings = ac_project.queryconf.config
    
        return inf_step
    