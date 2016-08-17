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
import numpy
import os
from pymongo import MongoClient
import prac
from prac.core.base import PRACModule, PRACPIPE, PRACDatabase
from prac.core.inference import PRACInferenceStep
from prac.pracutils.utils import prac_heading, get_query_png
from pracmln import praclog
from prac.db.ies.models import constants
from prac.db.ies.ies_utils.FrameSimilarity import frame_similarity

logger = praclog.logger(__name__, praclog.INFO)
corpus_path_list = os.path.join(prac.locations.home, 'corpus')


class ComplexAchievedBy(PRACModule):
    '''
    PRACModule used to perform action core refinement with a mongo database
    lookup.
    '''

    def get_howto_based_on_action_core(self, db):
        '''
        Determines the howto which describes how to perform the complex task.
        
        :param db: A PRAC database which represents the complex instruction.
        :return: A list of databases which each represents a step how to perform the complex instruction.
        '''

        # Assuming there is only one action core
        mongo_client = MongoClient(host=self.prac.config.get('mongodb', 'host'), 
                                   port=self.prac.config.getint('mongodb', 'port'))

        for q in db.query('achieved_by(?ac,Complex)'):
            actioncore = q['?ac']
            ies_mongo_db = mongo_client.prac
            instructions_collection = ies_mongo_db.howtos

            # ==================================================================
            # Mongo Lookup
            # ==================================================================

            logger.debug("Sending query to MONGO DB ...")
            cursor = instructions_collection.find({constants.JSON_HOWTO_ACTIONCORE: str(actioncore)})

            roles_dict = {}
            for ac2 in db.roles(actioncore=actioncore):
                roles_dict[ac2.keys()[0]] = ac2.values()[0]
            
            documents_vector = []

            # After the 'for loop' it is impossible to retrieve document by index
            cloned_cursor = cursor.clone()
            for document in cursor:
                documents_vector.append(frame_similarity(roles_dict,
                                                         document[constants.JSON_HOWTO_ACTIONCORE_ROLES]))
            if documents_vector:
                
                documents_vector = numpy.array(documents_vector)
                index = documents_vector.argmax()
                
                if documents_vector[index] > 0.75:
                    logger.debug('Found suitable howtos')
                    sub_dict = {}
                    steps = cloned_cursor[index][constants.JSON_HOWTO_STEPS]
                    document_action_roles = cloned_cursor[index][constants.JSON_HOWTO_ACTIONCORE_ROLES]
                    
                    #This module retrieves the howtos based on semantic.
                    #For instance, the howto "start the centrifuge" can be used to perform the task
                    #"start the mixer". Therefore it is required to replace the entities in the howto
                    #with the entities in the instruction.  
                    for role, sense in document_action_roles.iteritems():
                        if role == "action_verb":continue
                        if role in roles_dict.keys():
                            sub_dict[sense] = roles_dict[role]
            
                    if self.prac.verbose > 1:
                        print
                        print prac_heading('LOOKUP RESULTS')
                        print cloned_cursor[index]['_id']
                        
                    return map(lambda x: self.transform_step_to_db(x,sub_dict), steps)
                
                print "No suitable howtos are available."
        return []


    @PRACPIPE
    def __call__(self, pracinference, **params):

        # ======================================================================
        # Initialization
        # ======================================================================

        logger.debug('inference on {}'.format(self.name))

        if self.prac.verbose > 0:
            print prac_heading('Processing complex Action Core refinement')

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs

        pngs = {}
        for olddb in dbs:
            pracdb = PRACDatabase(self.prac, db=olddb)
            result = self.get_howto_based_on_action_core(pracdb)

            # ==================================================================
            # Postprocessing
            # ==================================================================

            if result:
                inf_step.output_dbs.extend(result)
            else:
                inf_step.output_dbs.append(olddb)

            for q in olddb.query('action_core(?w, ?ac)'):
                w = q['?w']

                pngs[q['?ac']] = get_query_png('instruction_sheet', dbs, filename=self.name, skolemword=w)
                inf_step.png = pngs

            inf_step.applied_settings = {'module': 'achieved_by', 'method': 'DB lookup'}
        return inf_step


    def transform_step_to_db(self, step, sub_dict):
        '''
        It transforms a step of a retrieved howto into a PRAC database.
        
        :param step: A single step of a retrieved howto represented as dictionary.
        :param sub_dict: Contains the information which synsets in the step should be replaced with which synsets in the instruction.
        :return: A PRAC database which represents a step to achieve the complex instruction. 
        '''
        
        i = 0
        db = PRACDatabase(self.prac)
        step_action_core = step[constants.JSON_FRAME_ACTIONCORE]
        step_action_roles = {}
        
        #Transform step action roles into directory
        for role in step[constants.JSON_FRAME_ACTIONCORE_ROLES]:
            step_action_roles[role] =  step[constants.JSON_FRAME_ACTIONCORE_ROLES][role][constants.JSON_SENSE_NLTK_WORDNET_SENSE]

        for role in step_action_roles.keys():
            sense = str(step_action_roles[role])
            #Substitute information in the step description with the information in the instruction.
            if sense in sub_dict.keys():
                sense = sub_dict[sense]
            
            word = "{}-{}mongo".format(sense.split('.')[0], str(i))
            
            db << ("has_pos({},{})".format(word, step[constants.JSON_FRAME_ACTIONCORE_ROLES]
                                           [role][constants.JSON_SENSE_PENN_TREEBANK_POS]))
            db << ("has_sense({},{})".format(word, sense))
            db << ("{}({},{})".format(str(role), word, step_action_core))
            if role == 'action_verb':
                db << ("action_core({},{})".format(word, step_action_core))
            i += 1
        
        return db
