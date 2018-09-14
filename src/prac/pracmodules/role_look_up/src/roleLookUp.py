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

from dnutils import logs, out

import prac
from prac.core.base import PRACModule
from prac.core.inference import PRACInferenceStep
from prac.db.ies.extraction import find_frames
from prac.db.ies.models import constants, Frame, Object
from prac.pracutils.utils import prac_heading, get_query_png


logger = logs.getlogger(__name__)


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
        rolenames = list(action_role.keys())

        for rolename in rolenames:
            document_dict[str(rolename)] = str(document[constants.JSON_FRAME_ACTIONCORE_ROLES]
                                                  [rolename]
                                                  [constants.JSON_OBJECT_TYPE])
        result.append(document_dict)
    return result


class RoleLookUp(PRACModule):
    '''
    PRACModule used determine missing roles by performing a mongo database
    look up. Will return most probable roles from previously stored
    instructions.
    '''

    def determine_missing_roles(self, node, db):
        '''
        Checks if the given database contains all required actionroles to 
        execute the representing actioncore successfully. If this is not the case
        this module look for frames which contain the missing action roles.

        :param db:
        :return:
        '''
        howtodb = self.prac.mongodb.prac.howtos
        db_ = db.copy()
        # Assuming there is only one action core
        actioncore = node.frame.actioncore
        # for word, actioncore in db.actioncores():
        # infer likely locations if a world model is given
        wm = node.pracinfer.worldmodel
        if wm:
            out('world model:')
            out(str(wm))
            for obj in node.frame.objects():
                out(obj.type)
                if obj.type not in wm.abstractions and not wm.contains(obj.type) and\
                        obj.props.__dict__.get('in') is None and obj.props.__dict__.get('on') is None:
                    out(obj.type, 'is not in world')
                    frames = find_frames(self.prac, actioncore='Storing', actionroles={'obj_to_be_stored': obj.type}, similarity=.7)
                    out('found', len(frames), 'potential locations:')
                    logger.debug('found', len(frames), 'potential locations:')
                    for frame, sim in frames:
                        out(sim, frame)
                        setattr(obj.props, 'in', frame.actionroles['location'].type)
                        break
                    if obj.props.__dict__.get('in'):
                        out('likely location:', obj.props.__dict__.get('in'))
                else:
                    out(obj.type, 'is in world')

        else:
            logger.debug('no world model given.')

        # ==================================================================
        # Preprocessing & Lookup
        # ==================================================================
        missingroles = node.frame.missingroles()#set(allroles).difference(givenroles)
        if 'action_verb' in missingroles:
            missingroles.remove('action_verb')
        # Build query: Query should return only frames which have the same actioncore as the instruction
        # and all required action roles
        if missingroles:
            and_conditions = [{'$eq' : ["$$plan.{}".format(constants.JSON_FRAME_ACTIONCORE), actioncore]}]
#                 and_conditions.extend([{"$ifNull" : ["$$plan.{}.{}".format(constants.JSON_FRAME_ACTIONCORE_ROLES, r), False]} for r in givenroles])
#                 and_conditions.extend([{"$eq" : ["$$plan.{}.{}.type".format(constants.JSON_FRAME_ACTIONCORE_ROLES, r), t]} for r, t in givenroles.items()])
            roles_query ={"$and" : and_conditions}

            stage_1 = {'$project' : {constants.JSON_HOWTO_STEPS: {
                                '$filter' :{
                                    'input': "${}".format(constants.JSON_HOWTO_STEPS),
                                    'as': "plan",
                                    'cond': roles_query
                            }
                        }, '_id': 0
                    }
            }
            stage_2 = {"$unwind": "${}".format(constants.JSON_HOWTO_STEPS)}

            if self.prac.verbose > 2:
                print("Sending query to MONGO DB ...")

            cursor_agg = howtodb.aggregate([stage_1, stage_2])

            # After once iterating through the query result
            # it is not possible to iterate again through the result.
            # Therefore we keep the retrieved results in a separate list.
            cursor = []
            for document in cursor_agg:
                cursor.append(document[constants.JSON_HOWTO_STEPS])
            for document in howtodb.find({constants.JSON_FRAME_ACTIONCORE: str(actioncore)}):
                cursor.append(document)
            frames = [Frame.fromjson(self.prac, d) for d in cursor]
            c = howtodb.find({constants.JSON_HOWTO_ACTIONCORE: str(actioncore)})
            frames.extend([Frame.fromjson(self.prac, d) for d in c])
            frames.sort(key=lambda f: f.specifity(), reverse=True)
            frames.sort(key=lambda f: node.frame.sim(f), reverse=True)
            if self.prac.verbose >= 2 or logger.level == logs.DEBUG:
                print('found similar frames in the db [%s]:' % str(node.frame))
                for f in frames:
                    print('%.2f: %s' % (node.frame.sim(f), f))
            if frames:
                frame = frames[0]
                if node.frame.sim(frame) >= node.pracinfer.similarity:
                    i = 0
                    for role in [m for m in missingroles if m in frame.actionroles]:
                        newword = "{}-{}-skolem-{}".format(frame.actioncore, role, str(i))
                        if self.prac.verbose:
                            print("Found {} as {}".format(frame.actionroles[role], role))
                        obj = frame.actionroles[role]
                        newobj = Object(self.prac, newword, obj.type, props=obj.props, syntax=obj.syntax)
                        node.frame.actionroles[role] = newobj
                        atom_role = "{}({}, {})".format(role, newword, actioncore)
                        atom_sense = "has_sense({}, {})".format(newword, obj.type)
                        atom_has_pos = "has_pos({}, {})".format(newword, obj.syntax.pos)
                        db_ << (atom_role, 1.0)
                        db_ << (atom_sense, 1.0)
                        db_ << (atom_has_pos, 1.0)

                        # Need to define that the retrieve role cannot be
                        # asserted to other roles
                        no_roles_set = set(self.prac.actioncores[actioncore].roles)
                        no_roles_set.remove(role)
                        for no_role in no_roles_set:
                            atom_role = "{}({},{})".format(no_role, newword, actioncore)
                            db_ << (atom_role, 0)
                        i += 1
                else:
                    if self.prac.verbose > 0:
                        print("Confidence is too low.")
            else:
                if self.prac.verbose > 0:
                    print("No suitable frames are available.")
        else:
            if self.prac.verbose > 0:
                print("No suitable frames are available.")
        return db_, missingroles

    def __call__(self, node, **params):
        # ======================================================================
        # Initialization
        # ======================================================================
        logger.debug('inference on {}'.format(self.name))

        if self.prac.verbose > 0:
            print(prac_heading('Role COMPLETION: %s' % node.frame.actioncore))

        dbs = node.outdbs
        infstep = PRACInferenceStep(node, self)
        infstep.executable_plans = []
        pngs = {}
        for i, db in enumerate(dbs):
            # ==================================================================
            # Mongo Lookup
            # ==================================================================
            infstep.indbs.append(db.copy())
            db_, missingroles = self.determine_missing_roles(node, db)
            if self.prac.verbose > 1:
                print()
                print(prac_heading('ROLE COMPLETION RESULTS'))
                for m in missingroles:
                    r = node.frame.actionroles.get(m)
                    if r:
                        print(m, r.type)
            # ==================================================================
            # Postprocessing
            # ==================================================================
            infstep.outdbs.append(db_)
            for word, actioncore in db.actioncores():
                pngs['LookUp - ' + str(i)] = get_query_png(list(missingroles),
                                                               dbs, filename=self.name,
                                                               skolemword=word)
            infstep.png = pngs
            infstep.applied_settings = {'module': 'missing_roles', 'method': 'DB lookup'}
        return [node]