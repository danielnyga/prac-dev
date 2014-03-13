# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012 by Daniel Nyga (nyga@cs.tum.edu)
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

from prac.core import PRACModule, PRACPIPE, PRACKnowledgeBase, DB_TRANSFORM
from nltk.corpus import wordnet as wn
import logging
from prac.inference import PRACInferenceStep
from mln import readMLNFromFile
import os
from mln.database import Database, readDBFromFile
from prac.wordnet import WordNet
from mln.util import mergeDomains
from collections import defaultdict

# mapping from PennTreebank POS tags to NLTK POS Tags
nounTags = ['NN', 'NNS', 'NNP']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'

class WNSenses(PRACModule):
    '''
    Extracts possible word senses from WordNet given the part of speech
    of a word. Depends on the 'syntax' feature extractor.
    '''
    
    def initialize(self):
        self.mln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'decls.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
        self.wordnetKBs = {}
        self.wordnet = self.prac.wordnet
    
    
    @DB_TRANSFORM
    def get_senses_and_similarities(self, db, concepts):
        '''
        Returns a new database with possible senses and the pairwise
        semantic similarities asserted.
        '''
        log = logging.getLogger(self.__class__.__name__)
        wordnet = self.wordnet
        word2senses = defaultdict(list)
        db_ = Database(self.mln)
        for res in db.query('has_pos(?word,?pos)'):
            word_const = res['?word']
            pos = posMap.get(res['?pos'], None)
            if pos is None:
                continue
            word = word_const.split('-')[0]
            for i, synset in enumerate(wordnet.synsets(word, pos)):
                sense_id = synset.name #'%s-%.2d' % (word_const, i+1)
                word2senses[word_const].append(sense_id)
                for concept in concepts:
#                     sim = wordnet.semilarity(synset, concept)
                    sim = wordnet.wup_similarity(synset, concept)
                    db_.addGroundAtom('is_a(%s,%s)' % (sense_id, concept), sim) 
        for word in word2senses:
            for word2, senses in word2senses.iteritems():
                if word2 == word: continue
                else: 
                    for s in senses: db_.addGroundAtom('!has_sense(%s,%s)' % (word, s))
            db_.addGroundAtom('!has_sense(%s,null)' % (word))
        for c in concepts:
            db_.addGroundAtom('!is_a(null,%s)' % c)
        return db_
    
    def get_similarities(self, *dbs):
        log = logging.getLogger(self.__class__.__name__)
        wordnet = self.wordnet
        full_domain = mergeDomains(*[db.domains for db in dbs])
        for db in dbs:
            db_ = Database(self.mln)
            for q in db.query('has_sense(?w, ?s) ^ is_a(?s, ?c)'):
                sense = q['?s']
                concept = q['?c']
                for c in full_domain['concept']:
                    sim = wordnet.wup_similarity(c, concept)
                    db_.addGroundAtom('is_a(%s,%s)' % (sense, c), sim)
            yield db_
            
    
    def addFuzzyEvidenceToDBs(self, *dbs):
        '''
        Adds to the databases dbs all fuzzy 'is_a' relationships
        for all senses contained in the DB and in the MLN.
        (has side effects on the original one)
        '''
        mln_domains = dbs[0].domains
        domains_full = mergeDomains(mln_domains, *[db.domains for db in dbs])
        concepts = domains_full['concept']
        wordnet = WordNet()
        log = logging.getLogger()
        for db in dbs:
            for res in db.query('is_a(?sense, ?concept)'):
                sense = res['?sense']
                concept = res['?concept']
                for c in concepts:
                    similarity = wordnet.semilarity(concept, c)
                    log.info('%s ~ %s = %.2f' % (concept, c, similarity))
                    db.addGroundAtom('is_a(%s,%s)' % (sense, c), similarity)
        return dbs
    
    def addPossibleWordSensesToDBs(self, *dbs):
        '''
        Adds to the databases dbs all possible word senses (and fuzzy meanings)
        based on their part of speech.
        '''
        log = logging.getLogger(self.__class__.__name__)
        wordnet = WordNet()
        for db in dbs:
            word2senses = defaultdict(list)
#             db.addGroundAtom('is_a(Nullsense,NULL)')
            log.info(db.mln.domains['concept'])
            for res in db.query('has_pos(?word,?pos)'):
                word_const = res['?word']
                pos = posMap.get(res['?pos'], None)
                if pos is None:
#                     db.addGroundAtom('has_sense(%s,Nullsense)' % word_const)
                    continue
                    #raise Exception('Invalid POS tag: %s' % res['?pos'])
                word = word_const.split('-')[0]
                for i, synset in enumerate(wordnet.synsets(word, pos)):
                    sense_id = '%s-%.2d' % (word_const, i+1)
                    word2senses[word_const].append(sense_id)
#                     print db.mln.domains['concept']
                    for concept in db.mln.domains['concept']:
#                         logging.getLogger('wn').info('%s -- %s' % (concept, synset.name))
                        sim = wordnet.semilarity(synset, concept)
                        db.addGroundAtom('is_a(%s,%s)' % (sense_id, concept), sim) 
            for word in word2senses:
                for word2, senses in word2senses.iteritems():
                    if word2 == word: continue
                    else: 
                        for s in senses: db.addGroundAtom('!has_sense(%s,%s)' % (word, s))
#                 db.addGroundAtom('!has_sense(%s,Nullsense)' % (word))
        
    
    @PRACPIPE
    def infer(self, pracinference):
        log = logging.getLogger('wnsenses')
        inf_step = PRACInferenceStep(pracinference, self)
        mt = self.load_pracmt('word_senses')
        for db in pracinference.get_inference_steps_of_module('nl_parsing').output_dbs:
            
            database = Database(mt.mln)
            for truth, gndLit in db.iterGroundLiteralStrings():
                database.addGroundAtom(gndLit, truth)
                log.info(gndLit)
            # default sense is the NULL-sense
#             database.addGroundAtom('is_a(Nullsense, NULL)')
            log.info('Adding all similarities...')
            self.addPossibleWordSensesToDBs(database)
            inf_step.output_dbs.append(database)
        return inf_step
    
    @PRACPIPE
    def train(self, prac_learning):
        
        training_dbs = []
        if hasattr(prac_learning, 'training_dbs') and prac_learning.training_dbs is not None:
            for dbfile in prac_learning.training_dbs:
                training_dbs.extend(readDBFromFile(self.mln, dbfile, ignoreUnknownPredicates=True))
        else:
            for dbfile in self.prac.getActionCoreTrainingDBs():
                db = readDBFromFile(self.mln, dbfile, ignoreUnknownPredicates=True)
                training_dbs.append(db)
        mt = WordSensesMT(self, 'word_senses')
        mt.train(training_dbs)
        self.save_pracmt(mt)
    
    
class WordSensesMT(PRACKnowledgeBase):
    '''
    Wrapper Knowledge Base around WordNet for pickling and unpickling.
    '''
    
    def train(self, training_dbs):
        self.mln = self.module.mln.duplicate()
        full_domains = mergeDomains(*[db.domains for db in training_dbs])
        logging.getLogger('wnsenses').debug('known concepts: %s' % full_domains['concept'])
        self.mln.domains['concept'] = full_domains['concept']
    