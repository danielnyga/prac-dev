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

from prac.core.base import PRACModule, PRACPIPE, PRACKnowledgeBase, DB_TRANSFORM
from nltk.corpus import wordnet as wn
import logging
from prac.core.inference import PRACInferenceStep
import os
from prac.core.wordnet import WordNet
from collections import defaultdict
from nltk.corpus.reader.wordnet import Synset

# mapping from PennTreebank POS tags to NLTK POS Tags
from pracmln import MLN, Database
from pracmln.mln.util import colorize, mergedom

nounTags = ['NN', 'NNS', 'NNP', 'CD']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
adjTags = ['JJ', 'JJR', 'JJS']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'
for a in adjTags:
    posMap[a] = 'a'


class WNSenses(PRACModule):
    '''
    Extracts possible word senses from WordNet given the part of speech
    of a word. Depends on the 'syntax' feature extractor.
    '''
    
    def initialize(self):
        self.mln = MLN(mlnfile=os.path.join(self.module_path, 'mln', 'predicates.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
        self.wordnetKBs = {}
        self.wordnet = self.prac.wordnet
    
    
    @DB_TRANSFORM
    def get_senses_and_similarities(self, db, concepts):
        '''
        Returns a new database with possible senses and the pairwise
        semantic similarities asserted. Assumes the part-of-speeches
        "has_pos" and the syntactic predicates from the parsing module
        to be given. Also asserts all negative evidences which cannot
        be true given the word senses from WordNet (eg, 'pancake' cannot
        have any meaning of milk).
        
        Example: "Flip the pancake."
        
        has_pos(pancake-3,NN) ---> 0.0  has_sense(pancake-3, Flip-1-sense-1)
                                   ....
                                   1.0  is_a(pancake-1-sense, pancake.n.01)
                                   0.6  is_a(pancake-1-sense, milk.n.01)
                                   ....
                                   
        All the reference concepts which are supposed to be considered
        need to be given by means of the 'concept' argument.
        NB: The existing databases are _not_ modified. Instead, copies
        of them are created and returned.
        
        '''
        log = logging.getLogger(self.__class__.__name__)
        wordnet = self.wordnet
        word2senses = defaultdict(list)
        db_ = db.copy()
        for res in db.query('has_pos(?word,?pos)'):
            word_const = res['?word']
            pos = posMap.get(res['?pos'], None)
            if pos is None: # if no possible sense can be determined by WordNet, assert null
                db_ << ('has_sense(%s,null)' % word_const)
                for c in concepts:
                    db_ << ('is_a(null,%s)' % c, 0)
                continue
            word = '-'.join(word_const.split('-')[:-1])# extract everything except the number (e.g. compound words like heart-shaped from heart-shaped-4)
            for i, synset in enumerate(wordnet.synsets(word, pos)):
                sense_id = synset.name#'%s-%.2d' % (word_const, i+1)
                word2senses[word_const].append(sense_id)
                for concept in concepts:
                    sim = wordnet.similarity(synset,concept)
#                     db_.addGroundAtom('is_a(%s,%s)' % (sense_id, concept), sim) 
#                     sim = wordnet.semilarity(synset, concept)
                    sim = wordnet.path_similarity(synset, concept)
                    db_ << ('is_a(%s,%s)' % (sense_id, concept),sim)
        for word in word2senses:
            for word2, senses in word2senses.iteritems():
                if word2 == word: continue
                else: 
                    for s in senses: 
                        db_ << ('!has_sense(%s,%s)' % (word, s))
            db_ << ('!has_sense(%s,null)' % (word))
#         for c in concepts:
#             db_.addGroundAtom('!is_a(null,%s)' % c)
        return db_
    
#     @DB_TRANSFORM
    def add_senses_and_similiarities_for_concepts(self, db, concepts):
        '''
        Adds for each concept in concepts a constant to the 'sense' domain
        and asserts all similarities to the other concepts for the 'is_a'
        predicate.
        Example:
        
        ``concepts = [pancake.n.01, spatula.n.01]``
        
        will be transformed into
        
        ``1.000  is_a(pancake-s-1, pancake.n.01)
          0.300  is_a(pancake-s-1, spatula.n.01)
          0.300  is_a(spatula-s-1, pancake.n.01)
          1.000  is_a(spatula-s-1, spatula.n.01)``
        '''
        db = db.copy()
        concepts = list(concepts)
        if 'null' in concepts:
            concepts.remove('null')
        for c in concepts:
            synset = self.wordnet.synset(c)
            sense_id = synset.name.lower().rsplit('.', 2)[0]
            for c2 in concepts:
                synset2 = self.wordnet.synset(c2)
                db << ('is_a(%s, %s)' % (synset.name, synset2.name), self.wordnet.similarity(synset, synset2))
        return db

    def add_similarities(self, db, domains, propsFound):
        '''
        For each property found in the evidence, we add one atom for each property of the same
        domain that can be found in the model. The degree of truth of the new atom is the similarity
        between the two properties (= wordnet concepts). This enables us to use previously unknown 
        concepts.

        Example: 
        If the evidence contains

        ``color(c, orange.s.01)'',

        the atoms 

        ``0.9 color(c, yellow.s.01)''
        ``0.95 color(c, red.s.01)''
        
        are added if yellow.s.01 and red.s.01 are in the color domain.

        '''
        db = db.copy()
        cluster = propsFound.pop('cluster', 'cluster')
        for prop in propsFound:
            for propFoundVal in propsFound[prop]:
                if prop in domains:
                    for domVal in domains[prop]:
                        if domVal not in propsFound[prop]:
                            if domVal not in [item for sublist in propsFound.values() for item in sublist]:
                                synset1 = self.wordnet.synset(propFoundVal)
                                synset2 = self.wordnet.synset(domVal)
                                sim = self.wordnet.similarity(synset1, synset2)
                                if prop == 'hasa' or prop == 'hypernym':
                                    if sim < .85: continue
                                    db << ('{}({}, {})'.format(prop, cluster, domVal), sim)
                                else:
                                    if sim < .6: continue
                                    db << ('{}({}, {})'.format(prop, cluster, domVal), sim)
        return db

           
    def printWordSenses(self, synsets, tick):
        '''
        Prints the list of synsets or synset ids and ticks the one specified by the given index.
        tick may be either the index or the sense id itself or the synset instance.
        '''
        if type(tick) is str:
            tick = self.prac.wordnet.synset(tick)
        synsets_ = []
        for idx, sense in enumerate(synsets):
            if isinstance(sense, str):
                sense = self.prac.wordnet.synset(sense)
            synsets_.append(sense)
        if isinstance(tick, Synset):
            tick = synsets_.index(tick) 
        for idx, sense in enumerate(synsets_):
            print '    [%s] %s: %s (%s)' % ('X' if tick==idx else ' ', colorize(sense.name, (None, {True: 'yellow', False: 'white'}[tick==idx], 
                                                                                             True), True), sense.definition, ';'.join(sense.examples))  

    
    def get_possible_meanings_of_word(self, db, word):
        '''
        Returns a list of synsets for the given word (the constant with index) in the
        given database. Assumes that parts-of-speech are known.
        '''
        for q in db.query('has_sense(%s, ?s) ^ has_pos(%s, ?pos)' % (word, word)):
            s = q['?s']
            pos = q['?pos']
            pos = posMap.get(pos, None)
            if pos is None: continue
            # make sure that words like bowl-shaped are left untouched
            # bowl-shaped-5 should become bowl-shaped and not bowl
            word = '-'.join(word.split('-')[:-1])
            return self.prac.wordnet.synsets(word, pos)
        return None
    
    
    def get_similarities(self, *dbs):
        log = logging.getLogger(self.__class__.__name__)
        wordnet = self.wordnet
        full_domain = mergedom(*[db.domains for db in dbs])
        for db in dbs:
            db_ = Database(self.mln)
            for q in db.query('has_sense(?w, ?s) ^ is_a(?s, ?c)'):
                sense = q['?s']
                concept = q['?c']
                for c in full_domain['concept']:
                    #sim = wordnet.wup_similarity(c, concept)
                    sim = wordnet.similarity(c, concept)
                    db_ << ('is_a(%s,%s)' % (sense, c), sim)
            yield db_
            
    
    def addFuzzyEvidenceToDBs(self, *dbs):
        '''
        Adds to the databases dbs all fuzzy 'is_a' relationships
        for all senses contained in the DB and in the MLN.
        (has side effects on the original one)
        '''
        mln_domains = dbs[0].domains
        domains_full = mergedom(mln_domains, *[db.domains for db in dbs])
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
                    db << ('is_a(%s,%s)' % (sense, c), similarity)
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
                        db << ('is_a(%s,%s)' % (sense_id, concept), sim)
            for word in word2senses:
                for word2, senses in word2senses.iteritems():
                    if word2 == word: continue
                    else: 
                        for s in senses: db << ('!has_sense(%s,%s)' % (word, s))
#                 db.addGroundAtom('!has_sense(%s,Nullsense)' % (word))
        
    
    @PRACPIPE
    def infer(self, pracinference):
        log = logging.getLogger('wnsenses')
        inf_step = PRACInferenceStep(pracinference, self)
        mt = self.load_pracmt('word_senses')
        for db in pracinference.get_inference_steps_of_module('nl_parsing').output_dbs:
            
            database = Database(mt.mln)
            for truth, gndLit in db.iterGroundLiteralStrings():
                database << (gndLit, truth)
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
                training_dbs.extend(Database(self.mln, dbfile=dbfile, ignoreUnknownPredicates=True))
        else:
            for dbfile in self.prac.getActionCoreTrainingDBs():
                db = Database(self.mln, dbfile=dbfile, ignoreUnknownPredicates=True)
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
        full_domains = mergedom(*[db.domains for db in training_dbs])
        logging.getLogger('wnsenses').debug('known concepts: %s' % full_domains['concept'])
        self.mln.domains['concept'] = full_domains['concept']
    
