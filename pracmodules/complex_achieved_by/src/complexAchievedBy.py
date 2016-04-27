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
from pracmln.mln.database import parse_db
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, out
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
from pymongo import MongoClient
from prac.pracutils.RolequeryHandler import RolequeryHandler
from scipy import stats
import numpy


log = logger(__name__)
PRAC_HOME = os.environ['PRAC_HOME']
corpus_path_list = os.path.join(PRAC_HOME, 'corpus')

def transform_to_db(plan_dict):
    db_str = str(plan_dict['DB'])
    mln_str = str(plan_dict['MLN'])
    mln = parse_mln(mln_str,logic='FuzzyLogic')
    dbs = parse_db(mln,db_str)
    #There should be only one db
    return dbs[0]
class ComplexAchievedBy(PRACModule):
    '''

    '''
    
    def get_senses_and_roles(self,actioncore,db):
        roles_db = RolequeryHandler.queryRoles(actioncore,db)
        
        inferred_roles_set = set()
        roles_dict = {}
        
        #Get inferred roles and the corresponding senses
        for atom, truth in sorted(roles_db.evidence.iteritems()):
            _ , predname, args = roles_db.mln.logic.parse_literal(atom)
            if truth == 1.0:
                inferred_roles_set.add(predname)
                for sense_query in db.query('has_sense({},?s)'.format(args[0])):
                    roles_dict[predname] = sense_query['?s']
        
        return roles_dict
                
    def get_instructions_based_on_action_core(self,db):
        wordnet = WordNet(concepts=None)
        #Assuming there is only one action core
        for q in db.query('action_core(?w,?ac)'):
            actioncore = q['?ac']
            mongo_client = MongoClient()
            ies_mongo_db = mongo_client.PRAC
            instructions_collection = ies_mongo_db.Instructions
            
            print "Sending query to MONGO DB ..."
            cursor = instructions_collection.find({'action_core' : '{}'.format(actioncore)})
            roles_dict =  self.get_senses_and_roles(actioncore, db)
            documents_vector = []
            
            #After the 'for loop' it is impossible to retrieve document by index
            cloned_cursor = cursor.clone()

            for document in cursor:
                print 'Found suitable instruction'
                wup_vector = []
                document_action_roles = document['action_roles']
                
                for role in roles_dict.keys():
                    if role in document['action_roles'].keys():
                        wup_vector.append(wordnet.wup_similarity(str(document_action_roles[role]),roles_dict[role]))
                
                if len(wup_vector) > 0 :
                    documents_vector.append(stats.hmean(wup_vector))
                else:
                    documents_vector.append(0)
            
            documents_vector = numpy.array(documents_vector)
            index = documents_vector.argmax()
            
            return map(lambda x : transform_to_db(x),cloned_cursor[index]['plan_list'])
            
            
        return []
            
    def initialize(self):
        pass

    def shutdown(self):
        pass

    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+================================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: PROCESSING COMPLEX ACHIEVED BY |  ' , (None, 'green', True), True)
        print colorize('+================================================+', (None, 'green', True), True)

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs
        
        
        for olddb in dbs:
            inf_step.output_dbs.extend(self.get_instructions_based_on_action_core(olddb))
            
        return inf_step
    
