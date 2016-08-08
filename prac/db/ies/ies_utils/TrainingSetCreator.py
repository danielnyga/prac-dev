'''
Created on Nov 10, 2015

@author: seba
'''
from mln.database import Database
from mln.mln import Predicate
from prac.core import PRAC
from prac.wordnet import WordNet
import sys
from ies_models import Sense,Constants
import os

prac = PRAC()
prac.wordnet = WordNet(concepts=None)
parser = prac.module('nl_parsing')
parser.initialize()
parser.mln.declarePredicate(Predicate('has_sense',['word','sense!']))
parser.mln.declarePredicate(Predicate('is_a',['sense','concept']))
parser.mln.declarePredicate(Predicate('action_core',['word','actioncore?']))
action_core = ""

def process_sentence(sentence):
    prac = PRAC()
    db = list(parser.parse_without_prac(sentence))[0]
    for q in db.query(Constants.HAS_POS_MLN_PREDICATE.format('?w','?p')):
        pos = Sense.convert_penn_treebank_pos_to_wordnet_pos(q['?p'])
        if pos != "unk":
            os.system("clear")
            print sentence
            word = '-'.join(q['?w'].split('-')[:-1])
            print word
            syns = Sense.get_synset(word, pos)
            i = 0
            for syn in syns:
                i += 1
                print "{}. {}".format(str(i),syn.definition)
            syn_id = int(raw_input())-1
            syn_name = syns[syn_id].name
            db.addGroundAtom(Constants.HAS_SENSE_MLN_PREDICATE.format(q['?w'], syn_name))
            db.addGroundAtom(Constants.IS_A_MLN_PREDICATE.format(syn_name, syn_name))
            
            print "Assert role"
            i = 0
            roles = prac.actioncores[action_core].roles
            for role in roles:
                i += 1
                print "{}. {}".format(str(i),role)
            role_id = int(raw_input())-1
            role_name = roles[role_id]
            db.mln.declarePredicate(Predicate(role_name,['word','actioncore']))
            db.addGroundAtom("{}({},{})".format(role_name,q['?w'],action_core))
            if pos == 'v':
                db.addGroundAtom("action_core({},{})".format(q['?w'],action_core))
            
    return db
            
if __name__ == '__main__':
    args = sys.argv[1:]
    if len(args) == 1:
        sentences_path = args[0]
        sentence_file = open(sentences_path,'r')
        sentences  = sentence_file.readlines()
        dbs = []
        action_core = raw_input("Enter Action Core: ")
        for sentence in sentences:
            dbs.append(process_sentence(sentence))
        
        dbs_file = open("{}.db".format(sentences_path),'w')
        Database.writeDBs(dbs,dbs_file)
        dbs_file.close()