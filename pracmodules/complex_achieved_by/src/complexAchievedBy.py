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


def transform_to_db(complex_db, roles_dict, document_action_roles, actioncore,
                    plan_dict):
    plan_action_core = plan_dict['action_core']
    plan_action_roles = plan_dict['action_roles']

    mln_str = str(plan_dict['MLN'])
    mln = parse_mln(mln_str, logic='FuzzyLogic')
    i = 0
    db = Database(mln=mln)

    for action_role in plan_action_roles.keys():
        sense = ""
        word = ""
        pos = ""

        updated_role = False
        if plan_action_core == 'Pipetting' and actioncore == 'Evaluating' and action_role == 'content':
            for q in complex_db.query(
                    'obj_to_be_evaluated(?w,?ac) ^ has_sense(?w,?s) ^ has_pos(?w,?p) '):
                sense = q["?s"]
                word = q["?w"]
                pos = q["?p"]
                updated_role = True

        elif plan_action_core == 'Pressing' and actioncore == 'Starting' and action_role == 'location':
            for q in complex_db.query(
                    'obj_to_be_started(?w,?ac) ^ has_sense(?w,?s) ^ has_pos(?w,?p) '):
                sense = q["?s"]
                word = q["?w"]
                pos = q["?p"]
                updated_role = True

        elif not updated_role:
            sense = str(plan_action_roles[action_role])
            splitted_sense = sense.split('.')
            if splitted_sense[1] == 'v':
                pos = 'VB'
            else:
                pos = 'NN'

            word = "{}-{}mongo".format(splitted_sense[0], str(i))

        db << ("has_pos({},{})".format(word, pos))
        db << ("has_sense({},{})".format(word, sense))
        db << ("{}({},{})".format(str(action_role), word, plan_action_core))
        if action_role == 'action_verb':
            db << ("action_core({},{})".format(word, actioncore))
        i += 1

    db << ("achieved_by({},{})".format(actioncore, plan_action_core))

    return db


class ComplexAchievedBy(PRACModule):
    '''

    '''


    def get_instructions_based_on_action_core(self, db):
        wordnet = WordNet(concepts=None)
        # Assuming there is only one action core
        for q in db.query('action_core(?w,?ac)'):
            actioncore = q['?ac']
            mongo_client = MongoClient()
            ies_mongo_db = mongo_client.PRAC
            instructions_collection = ies_mongo_db.Instructions

            out("Sending query to MONGO DB ...")
            cursor = instructions_collection.find(
                {'action_core': '{}'.format(actioncore)})
            roles_dict = RolequeryHandler(self.prac).query_roles_and_senses_based_on_achieved_by(db)
            documents_vector = []

            # After the 'for loop' it is impossible to retrieve document by index
            cloned_cursor = cursor.clone()

            for document in cursor:
                wup_vector = []
                document_action_roles = document['action_roles']

                for role in roles_dict.keys():
                    if role in document['action_roles'].keys():
                        wup_vector.append(wordnet.wup_similarity(
                            str(document_action_roles[role]),
                            roles_dict[role]))

                if len(wup_vector) > 0:
                    documents_vector.append(stats.hmean(wup_vector))
                else:
                    documents_vector.append(0)

            if documents_vector:
                out('Found suitable instruction')
                documents_vector = numpy.array(documents_vector)
                index = documents_vector.argmax()

                return map(lambda x: transform_to_db(db, roles_dict,
                                                     document_action_roles,
                                                     actioncore, x),
                           cloned_cursor[index]['plan_list'])

        return []


    def initialize(self):
        pass


    def shutdown(self):
        pass


    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+================================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: PROCESSING COMPLEX ACHIEVED BY |  ',
                       (None, 'green', True), True)
        print colorize('+================================================+',
                       (None, 'green', True), True)

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs

        pngs = {}
        for olddb in dbs:
            result = self.get_instructions_based_on_action_core(olddb)
            if result:
                inf_step.output_dbs.extend(result)
            else:
                inf_step.output_dbs.append(olddb)

            for q in olddb.query('action_core(?w, ?ac)'):
                w = q['?w']

                pngs[q['?ac']] = get_cond_prob_png('instruction_sheet', dbs, filename=self.name, mongo=True, mongoword=w)
                inf_step.png = pngs

            inf_step.applied_settings = {'module': 'achieved_by',
                                         'method': 'DB lookup'}
        return inf_step
