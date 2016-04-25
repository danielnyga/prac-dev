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
from pymongo import MongoClient
from prac.pracutils.RolequeryHandler import RolequeryHandler
from prac.pracutils.ActioncoreDescriptionHandler import ActioncoreDescriptionHandler
from scipy import stats
import numpy


log = logger(__name__)
PRAC_HOME = os.environ['PRAC_HOME']
corpus_path_list = os.path.join(PRAC_HOME, 'corpus')

def transform_to_frame_vector(inferred_roles,frame_action_role_dict):
    wordnet = WordNet(concepts=None)
    frame_vector = []
    
    for role, sense in inferred_roles.iteritems():
        if role in frame_action_role_dict.keys():
            frame_vector.append(wordnet.wup_similarity(frame_action_role_dict[role],sense))
    
    return stats.hmean(frame_vector)

def transform_documents_to_action_role_map(cursor):
    result = []
    
    for document in cursor:
        document_map = {}
        action_role = document['actioncore_roles']
        key_list = action_role.keys()
        
        for key_element in key_list:
            document_map[str(key_element)] = str(document['actioncore_roles'][key_element]['nltk_wordnet_sense'])
        
        result.append(document_map)
    
    return result 
class RoleLookUp(PRACModule):
    '''

    '''
    def determine_missing_roles(self,db):
        mongo_client = MongoClient()
        ies_mongo_db = mongo_client.IES
        frames_collection = ies_mongo_db.Frames
        
        db_ = db.copy()
        #Assuming there is only one action core
        for q in db.query('action_core(?w,?ac)'):
            
            actioncore = q['?ac']
            roles_senses_dict = RolequeryHandler.query_roles_and_senses_based_on_action_core(db_)
            inferred_roles_set = set(roles_senses_dict.keys())
            
            #Determine missing roles: All_Action_Roles\Inferred_Roles
            actioncore_roles_list = ActioncoreDescriptionHandler.get_required_roles_based_on_actioncore(actioncore)
            missing_role_set = set(actioncore_roles_list).difference(inferred_roles_set)
            
            #Build query, return only frames where all roles are defined
            
            if missing_role_set:
                roles_query = [{"action_core" : "{}".format(actioncore)}]
                roles_query.extend(map(lambda x: {"actioncore_roles.{}".format(x) : {'$exists': 'true'}}, actioncore_roles_list))
                
                #build query based on inferred senses and roles
                cursor = frames_collection.find({'$and' : roles_query})
                if cursor.count() > 0:
                    frame_result_list = transform_documents_to_action_role_map(cursor)
                    score_frame_matrix = numpy.array(map(lambda x: transform_to_frame_vector(roles_senses_dict,x),frame_result_list))
                    confidence_level = 0.7
                    
                    argmax_index = score_frame_matrix.argmax()
                    current_max_score = score_frame_matrix[argmax_index]
                        
                    if current_max_score >= confidence_level:
                        frame = frame_result_list[argmax_index]
                        
                        i = 0
                        for missing_role in missing_role_set:
                            print "Found {} as {}".format(frame[missing_role],missing_role)
                            atom_role = "{}({},{})".format(missing_role,"word_mongo_{}".format(str(i)),actioncore)
                            atom_sense = "{}({},{})".format('has_sense',"word_mongo_{}".format(str(i)),frame[missing_role])
                            
                            db_ << (atom_role,1.0)
                            db_ << (atom_sense,1.0)
                            i += 1
                    else:
                        print "Confidence is too low."
                        
                else:
                    print "No suitable frames are available."
                        
        return db_
    
    def initialize(self):
        pass

    def shutdown(self):
        pass

    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+==============================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: ROLE LOOK UP |  ' , (None, 'green', True), True)
        print colorize('+==============================+', (None, 'green', True), True)

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs
        inf_step.executable_plans = []
        
        for db in dbs:
            inf_step.output_dbs.append(self.determine_missing_roles(db))
            
        return inf_step