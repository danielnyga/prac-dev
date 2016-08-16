# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2015 by Daniel Nyga (nyga@cs.tum.edu)
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
from pymongo import MongoClient
from prac.db.ies.ies_utils.FrameSimilarity import frame_similarity
import numpy

import prac
from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from prac.pracutils.utils import prac_heading, get_query_png
from pracmln import praclog

from prac.db.ies.models import constants


logger = praclog.logger(__name__, praclog.INFO)
corpus_path_list = os.path.join(prac.locations.home, 'corpus')




def transform_documents_to_actionrole_dict(cursor):
    '''
    Since it is only neccessary to have the actionroles of the retrieved frames to determine the frame similarity, 
    this function transforms the frames retrieved from the MongoDB 
    into a list of dictionaries which have the form : role_name : nltk_wordnet_sense.

    :param cursor: Represents the list of frames retrieved from the MongoDB
    :return: A list of dictionaries which each represents the actionroles of a specific frame.
    '''
    result = []

    for document in cursor:
        document_dict = {}
        action_role = document[constants.JSON_FRAME_ACTIONCORE_ROLES]
        key_list = action_role.keys()

        for key_element in key_list:
            document_dict[str(key_element)] = str(document[constants.JSON_FRAME_ACTIONCORE_ROLES]
                                                  [key_element]
                                                  [constants.JSON_SENSE_NLTK_WORDNET_SENSE])
        
        result.append(document_dict)
        
    return result


class RoleLookUp(PRACModule):
    '''
    PRACModule used determine missing roles by performing a mongo database
    look up. Will return most probable roles from previously stored
    instructions.
    '''

    def determine_missing_roles(self, db):
        '''
        Checks if the given database contains all required actionroles to 
        execute the representing actioncore successfully. If this is not the case
        this module look for frames which contain the missing action roles.

        :param db:
        :return:
        '''
        mongo_client = MongoClient(host=self.prac.config.get('mongodb', 'host'), 
                                   port=self.prac.config.getint('mongodb', 'port'))
        ies_mongo_db = mongo_client.prac
        frames_collection = ies_mongo_db.howtos

        db_ = db.copy()
        # Assuming there is only one action core
        for q in db.query('action_core(?w,?ac)'):

            # ==================================================================
            # Preprocessing & Lookup
            # ==================================================================

            actioncore = q['?ac']
            #Represent the inferred roles from the instruction as a dictionary 
            roles_senses_dict = {k: v for r in db.roles(actioncore) for k, v in r.items()}
            
            
            actioncore_roles_list = self.prac.actioncores[actioncore].required_roles
            if not actioncore_roles_list:
                actioncore_roles_list = self.prac.actioncores[actioncore].roles
            
            # Determine missing roles: All_Action_Roles\Inferred_Roles
            missing_role_set = set(actioncore_roles_list).difference(set(roles_senses_dict.keys()))

            # Build query: Query should return only frames which have the same actioncore as the instruction 
            # and all required action roles
            if missing_role_set:
                and_conditions = [{'$eq' : ["$$plan.{}".format(constants.JSON_FRAME_ACTIONCORE), 
                                            "{}".format(actioncore)]}]
                and_conditions.extend(map(lambda x: {"$ifNull" : ["$$plan.{}.{}".format(constants.JSON_FRAME_ACTIONCORE_ROLES,x),False]},
                                          actioncore_roles_list))
                
                roles_query ={"$and" : and_conditions}                
                
                stage_1 = {'$project' : {
                            '{}'.format(constants.JSON_HOWTO_STEPS) : {
                                '$filter' :{
                                    'input':"${}".format(constants.JSON_HOWTO_STEPS),
                                    'as':"plan",
                                    'cond': roles_query
                                }
                            },'_id':0
                            }
                           }
                
                stage_2 = {"$unwind": "${}".format(constants.JSON_HOWTO_STEPS)}

                if self.prac.verbose > 0:
                    print "Sending query to MONGO DB ..."

                cursor_agg = frames_collection.aggregate([stage_1, stage_2])
                
                # After once iterating through the query result 
                #it is not possible to iterate again through the result.
                #Therefore we keep the retrieved results in a separate list.
                cursor = []
                for document in cursor_agg:
                    cursor.append(document[constants.JSON_HOWTO_STEPS])
                
                if len(cursor) > 0:
                    frame_result_list = transform_documents_to_actionrole_dict(cursor)
                    
                    if len(frame_result_list) > 0:
                        logger.info("Found suitable frames")
                        score_frame_matrix = numpy.array(map(lambda x: frame_similarity(roles_senses_dict, x), 
                                                             frame_result_list))
                        confidence_level = 0.7
    
                        argmax_index = score_frame_matrix.argmax()
                        current_max_score = score_frame_matrix[argmax_index]
    
                        if current_max_score >= confidence_level:
                            frame = frame_result_list[argmax_index]
                            document = cursor[argmax_index]
                            i = 0
                            for missing_role in missing_role_set:
                                word = "{}mongo{}".format(str(document[constants.JSON_FRAME_ACTIONCORE_ROLES]
                                                              [missing_role]
                                                              [constants.JSON_SENSE_WORD]), str(i))
                                print "Found {} as {}".format(frame[missing_role], missing_role)
                                atom_role = "{}({}, {})".format(missing_role, word, actioncore)
                                atom_sense = "{}({}, {})".format('has_sense', word, frame[missing_role])
                                atom_has_pos = "{}({}, {})".format('has_pos', word, str(document[constants.JSON_FRAME_ACTIONCORE_ROLES]
                                                                                        [missing_role]
                                                                                        [constants.JSON_SENSE_PENN_TREEBANK_POS]))
    
                                db_ << (atom_role, 1.0)
                                db_ << (atom_sense, 1.0)
                                db_ << (atom_has_pos, 1.0)
    
                                # Need to define that the retrieve role cannot be
                                # asserted to other roles
                                no_roles_set = set(self.prac.actioncores[actioncore].roles)
                                no_roles_set.remove(missing_role)
                                for no_role in no_roles_set:
                                    atom_role = "{}({},{})".format(no_role, word, actioncore)
                                    db_ << (atom_role, 0)
                                i += 1
                        else:
                            if self.prac.verbose > 0:
                                print "Confidence is too low."
                    else:
                        if self.prac.verbose > 0:
                            print "No suitable frames are available."
                else:
                    if self.prac.verbose > 0:
                        print "No suitable frames are available."
        
        return db_, missing_role_set


    @PRACPIPE
    def __call__(self, pracinference, **params):

        # ======================================================================
        # Initialization
        # ======================================================================

        logger.debug('inference on {}'.format(self.name))

        if self.prac.verbose > 0:
            print prac_heading('Role Look-up')

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs
        inf_step.executable_plans = []

        pngs = {}
        for i, db in enumerate(dbs):

            # ==================================================================
            # Mongo Lookup
            # ==================================================================

            db_, missingroles = self.determine_missing_roles(db)

            if self.prac.verbose > 1:
                print
                print prac_heading('LOOKUP RESULTS')
                for m in missingroles:
                    print m

            # ==================================================================
            # Postprocessing
            # ==================================================================

            inf_step.output_dbs.append(db_)

            for q in db.query('action_core(?w, ?ac)'):
                w = q['?w']

                pngs['LookUp - ' + str(i)] = get_query_png(list(missingroles),
                                                               dbs, filename=self.name,
                                                               skolemword=w)
            inf_step.png = pngs
            inf_step.applied_settings = {'module': 'missing_roles', 'method': 'DB lookup'}
        return inf_step
