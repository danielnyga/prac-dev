'''
This class represents a frame.
The idea of a frame is inspired by IBM.

Created on Sep 2, 2015

@author: Sebastian Koralewski (seba@informatik.uni-bremen.de)
'''

from prac.db.ies.models import constants
import cStringIO
from pracmln.mln.database import Database
import sys
from prac.core.inference import PRACInference,PRACInferenceStep
from scipy import stats
from prac.core.wordnet import WordNet
import traceback
import json


class Frame(object):
    '''
    classdocs
    '''

    def __init__(self,
                 text_source_file,
                 sentence_number,
                 frame_id,
                 sentence,
                 syntax,
                 actioncore=None,
                 actioncore_roles = {},
                 prac=None,
                 process_frame = True):
        '''
        Constructor
        '''
        self.text_source_file = text_source_file
        self.sentence_number = sentence_number
        self.frame_id = frame_id
        self.sentence = sentence
        self.syntax = syntax
        self.prac = prac
        #If documents are convert to frames we do not need an Actioncore inference process
        if process_frame:
            self.actioncore_roles = {}
            self.actioncore = "UNKNOWN"
            try:
                self.handle_prac_inference()
            except:
                traceback.print_exc()
                self.actioncore = "UNKNOWN"
        else:
            self.actioncore_roles = actioncore_roles
            self.actioncore = actioncore


    def handle_prac_inference(self):
        infer = PRACInference(self.prac, 'None')
        self.try_to_determine_actioncore(self.prac, infer)    
        self.try_to_determine_action_roles(self.prac, infer)


    def try_to_determine_actioncore(self,prac,infer):
        self.actioncore = "UNKNOWN"
        
        ac_recognition_module = prac.module('ac_recognition')
        ac_recognition_module.initialize()
        
        infer_step = PRACInferenceStep(infer, ac_recognition_module)
        infer.inference_steps.append(infer_step)
        infer.inference_steps[0].output_dbs.append(self.syntax_to_prac_db(False))
        prac.run(infer,ac_recognition_module,kb=None)
        result_db = infer.inference_steps[-1].output_dbs[0]
        
        for q in result_db.query('{}(?w,?ac)'.format("action_core")):
                self.actioncore = q['?ac']


    def try_to_determine_action_roles(self, prac, infer):
        senses_and_roles_module = prac.module('senses_and_roles')
        senses_and_roles_module.initialize()
        prac.run(infer,senses_and_roles_module,kb=None)
        result_db = infer.inference_steps[-1].output_dbs[0]
        
        role_predicate_list = []
        
        for result_ac in result_db.actioncores():
            
            for e in result_db.roles(result_ac.values().pop()):
                role_predicate_list.extend(e.keys())

        senses_dict = {}
        roles_dict = {}
        
        #store senses in dict
        for q in result_db.query(constants.HAS_SENSE_MLN_PREDICATE.format('?w','?s')):
                senses_dict[q['?w']] = q['?s']
        
        #store roles in dict
        for atom, truth in sorted(result_db.evidence.iteritems()):
            _, predname, args = result_db.mln.logic.parse_literal(atom)
            if (truth == 1.0) and (predname in role_predicate_list):
                roles_dict[args[0]] = predname
        
        #Senses Assertion to slot objects and define action_core_roles
        for _, value in self.syntax.iteritems():
            if value is None: continue
            if value.word in senses_dict.keys():
                value.nltk_wordnet_sense = senses_dict[value.word]
                if value.word in roles_dict.keys():
                    self.actioncore_roles[roles_dict[value.word]] = value 
        
    
    @property
    def json(self):
        return {constants.JSON_FRAME_ID : '%s#%d#%d' % (self.text_source_file, self.sentence_number, self.frame_id),
                constants.JSON_FRAME_SENTENCE: self.sentence,
                constants.JSON_FRAME_ACTIONCORE: self.actioncore,
                constants.JSON_FRAME_SYNTAX: self.syntax_to_json(),
                constants.JSON_FRAME_ACTIONCORE_ROLES: self.actioncore_roles_to_json_str()}

    
    def syntax_to_prac_db(self,add_senses=True):
        db = Database(self.prac.mln)
        atom_list = []
        sv_predicate = self.syntax[constants.SLOT_VALUE_PREDICATE]
        atom_list.append(constants.HAS_POS_MLN_PREDICATE.format(sv_predicate.word, sv_predicate.penn_treebank_pos))
        
        for key, value in self.syntax.iteritems():
            if value is not None and key != constants.SLOT_VALUE_PREDICATE:
                #Handle prepobj to extract the correct preposition e.g prep_with
                if key == constants.SLOT_VALUE_PREPOBJ:
                    atom_list.append("nmod_{}({},{})".format(value.misc, sv_predicate.word,value.word))  
                else:
                    #Create dependencies predicates
                    atom_list.append("{}({},{})".format(key,sv_predicate.word,value.word))
                atom_list.append(constants.HAS_POS_MLN_PREDICATE.format(value.word, value.penn_treebank_pos))
                #Create sense related predicates
                if value.nltk_wordnet_sense and add_senses:
                    atom_list.append(constants.HAS_SENSE_MLN_PREDICATE.format(value.word,value.nltk_wordnet_sense))
                    atom_list.append(constants.IS_A_MLN_PREDICATE.format(value.nltk_wordnet_sense, value.nltk_wordnet_sense))
        for atom in atom_list:
            db << atom
        return db
        
        
    def actioncore_roles_to_json_str(self):
        return self.convert_senses_dict_to_json_str(self.actioncore_roles)
        
        
    def syntax_to_json_str(self):
        return self.convert_senses_dict_to_json_str(self.syntax)
        

    def convert_senses_dict_to_json_str(self, senses_dict):
        num_of_syntax = len(senses_dict.keys())

        syntax_as_json = ""
        
        for i in range(0, num_of_syntax):
            slot = senses_dict.keys()[i]
            value = senses_dict[slot]
            value_as_str = '{}'
            
            if value is not None:
                value_as_str = value.json
                
            syntax_as_json = syntax_as_json + '"{}" : {}'.format(slot,value_as_str)
            
            if i < (num_of_syntax-1):
                syntax_as_json = syntax_as_json + ","
        return "{{{}}}".format(syntax_as_json)
    
        
    def is_valid(self):
        i = 0
        #predicate and one other slot value have not to be empty
        for _, slot_value in self.syntax.iteritems():
            if slot_value:
                i += 1
        if i > 1:
            return True
        return False
    
    
    def transform_to_frame_vector(self,inferred_roles,missing_roles):
        if not self.actioncore_roles:
            return 0
        wordnet = WordNet(concepts=None)
        frame_vector = []
        for role, sense in inferred_roles.iteritems():
            if role in self.actioncore_roles.keys():
                sim_result = wordnet.wup_similarity(self.actioncore_roles[role].nltk_wordnet_sense,sense)
                if role == 'action_verb' and sim_result < 0.85:
                    return 0
                frame_vector.append(sim_result)
        if not missing_roles:
            return stats.hmean(frame_vector)
        if len(frame_vector) > 1 :
            for m_role in missing_roles:
                if m_role in self.actioncore_roles.keys():
                    return stats.hmean(frame_vector)
            return 0
        else:
            return 0
    
        