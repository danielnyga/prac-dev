'''
Created on Sep 2, 2015

@author: seba
'''

import traceback
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
import sys
from prac.db.ies.models import Frame, Word, Howto

from pymongo import MongoClient
import pymongo
import json
from pracmln.mln.errors import NoSuchPredicateError
from pracmln.mln.base import Predicate
import os
from prac.db.ies.models import constants
from prac.db.ies.exceptions import NoPredicateExtracted, NoValidFrame
from pracmln.mln.util import out
from collections import defaultdict
from pprint import pprint


def __dividedict(d, dnew):
    if not d: 
        yield dnew
        return
    key, values = d.popitem()
    for v in values:
        dnew_ = dict(dnew)
        dnew_[key] = v
        for d_ in __dividedict(dict(d), dnew_): yield d_

def ddivide(d):
    return __dividedict(d, {})


class HowtoImport(object):
    '''
    '''

    def __init__(self, prac, howto):
        self.howto = howto
        self.prac = prac


    def run(self):
        instr, steps = dict(self.howto).popitem()
        howto = self.buildhowto(instr, steps)
        if howto is not None:
            pprint(howto.tojson())
#             self.prac.mongodb.prac.howtos.insert_one(howto_.tojson())
    
    
    def buildframes(self, db, sidx, sentence):
        for p, args in db.syntax():
            print p, args
        for _, actioncore in db.actioncores():
            roles = defaultdict(list)
            for role, sense in db.roles(actioncore):
                roles[role].append(sense)
            frames = ddivide(roles)    
            for frame in frames:
                yield Frame(sidx, sentence, syntax=list(db.syntax()), words=self.buildwords(db), actioncore=actioncore, actionroles=frame)
            
            
    def buildwords(self, db):
        for word in db.words():
            tokens = word.split('-')
            w = '-'.join(tokens[:-1])
            idx = int(tokens[-1])
            pos = set(db.postag(word)).pop()
            sense = db.sense(word)
            nltkpos = db.prac.wordnet.nltkpos(pos)
            lemma = db.prac.wordnet.lemmatize(w, nltkpos) if nltkpos is not None else None
            yield Word(word, w, idx, sense, pos, lemma)
            
        
        
    def buildhowto(self, howto, steps):
        stopmodules = ('role_look_up', 'achieved_by', 'complex_achieved')
        mainresult = self.prac.query(howto, stopat=stopmodules).inference_steps[-1].output_dbs
        stepresults = self.prac.query(steps, stopat=stopmodules).inference_steps[-1]
        mainframe = self.buildframes(mainresult[0], 0, howto)
        frames = [] 
        for i, step in enumerate(stepresults.output_dbs):
            frames.extend(self.buildframes(step, i, steps[i]))
        for instr in mainframe: break
        return Howto(instr=instr, steps=frames)
    
    
    def store_frames_into_database(self, howto, frames):
        pracdb = self.prac.mongodb.prac
        howtos = pracdb.howtos
        steps = []
        actioncore = None
        roles = {}
        try:
            #Parse text file name to annotate it in the mongo db
            inference = PRACInference(self.prac, ["{}.".format(os.path.basename(howto))])
            
            while inference.next_module() not in ('role_look_up', 'achieved_by', 'plan_generation'):
                modulename = inference.next_module()
                module = self.prac.module(modulename)
                self.prac.run(inference, module)
        
            db = inference.inference_steps[-1].output_dbs[0]
            
            for result_ac in db.actioncores():
                for result_role in db.roles(result_ac.values().pop()):
                    roles[result_role.keys()[0]] = result_role.values()[0]
            # we assume that there is only one true action_core predicate per database 
            for _, actioncore in db.actioncores(): break
        except:
            actioncore = None
        for frame in frames:
            steps.append(frame.json)
        try:
            document = {'_id' : text_file_name,
                        constants.JSON_HOWTO_ACTIONCORE : actioncore, 
                        constants.JSON_HOWTO_ACTIONCORE_ROLES : roles,
                        constants.JSON_HOWTO_STEPS : steps}
            howtos.insert_one(document)
        except pymongo.errors.DuplicateKeyError:
            howtos.delete_many({"_id" : document['_id']})
            howtos.insert_one(document)
        except:
            traceback.print_exc()
            

    