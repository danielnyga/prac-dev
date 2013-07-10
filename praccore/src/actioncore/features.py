# PROBABILISTIC ROBOT ACTION CORES - FEATURES
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

import java
import os
from logic.grammar import parseFormula 
from nltk.corpus import wordnet as wn
from logic import FOL
from utils import list_get



java.classpath.append(os.path.join('3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))

# mapping from PennTreebank POS tags to NLTK POS Tags
nounTags = ['NN', 'NNS', 'NNP']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'

class FeatureManager(object):
    
    def __init__(self, prac, features=None):
        self.features = {}
        self.prac = prac
        if features is not None:
            for f in features:
                self.features[f.name] = f
 
    def add(self, feature):
        self.features[feature.name] = feature
    
    def get(self, name):
        return self.features.get(name, None)

class FeatureExtractor(object):
    '''
    Abstract class for extracting features out of a natural-language sentence.
    '''
    
    def __init__(self, name, mngr):
        '''
        Initializes the feature extractor. The name uniquely identifies the
        feature extractor (Used for dependencies).
        '''
        self.name = name
        self.mngr = mngr
        self.evidence = []
        
    def run(self, pracinference):
        '''
        Runs the feature extraction on a sentence. The database contains 
        the features collected so far.
        '''
        pass
    
    def getEvidence(self):
        '''
        Returns the list of ground atoms that have been extracted by this
        feature extractor.
        '''
        return self.evidence
    
    @staticmethod
    def addEvidence(db, mln, evidence):
        '''
        Adds evidence to the db or the mln. eividence may be a ground atom
        or a formula.
        '''
        if isinstance(evidence, basestring):
            evidence = parseFormula(evidence)
        if isinstance(evidence, FOL.Lit):
            db.addGroundAtom(str(evidence))
        elif isinstance(evidence, FOL.Formula):
            mln.addFormula(evidence, hard=True)
            
        
class Syntax(FeatureExtractor):
    '''
    Extracts syntactic features that are obtained by the parser, such as
    'has_pos' and the stanford dependencies.
    '''

    def __init__(self, mngr):
        FeatureExtractor.__init__(self, 'syntax', mngr)    

    def run(self, pracinference):
        self.parser = pracinference.synParser
        deps = self.parser.getDependencies(pracinference.sentence, True)
        self.deps = map(str, deps)
        words = set()
        for d in deps:
            f = parseFormula(str(d))
            words.update(f.params)
        self.posTags = self.parser.getPOS()
        self.pos = []
        for pos in self.posTags.values():
            if not pos[0] in words:
                continue
            self.pos.append('has_pos(%s,%s)' % (pos[0], pos[1]))
            self.posTags[pos[0]] = pos[1]
        self.evidence = self.pos + self.deps
        for e in self.evidence:
            FeatureExtractor.addEvidence(pracinference.databases['core'], pracinference.mlns['pracinit'], e)
    
class WordSenses(FeatureExtractor):
    '''
    Extracts possible word senses from WordNet given the part of speech
    of a word. Depends on the 'syntax' feature extractor.
    '''
    
    def __init__(self, mngr):
        FeatureExtractor.__init__(self, 'wordsenses', mngr)
        self.evidence = []
        
    def run(self, pracinference):
        # here we need the part of speeches
        database = pracinference.databases['core']
        mln = pracinference.mlns['pracinit']
        syntax = self.mngr.get('syntax')
        if syntax is None:
            raise Exception('"wordsenses" depends on "syntax" features. Run syntax features first.')
        
        # default sense is the NULL-sense
        FeatureExtractor.addEvidence(database, mln, 'is_a(Nullsense, NULL)')
        
        # map from word constants to possible senses
        self.words2senses = {}
        # map from sense constants to taxonomic concepts
        self.senses2concepts = {}
        # map from sense constants to hypernym paths
        self.senses2hypernyms = {}
        for i, word in enumerate(database.domains['word']):
            disjuncts = []
            tokens = word.rsplit('-', 1)
            nltkpos = posMap.get(syntax.posTags[word], None)
            
            if nltkpos is None:
                FeatureExtractor.addEvidence(database, mln, 'has_sense(%s,Nullsense)' % word)
                continue
            
            # go through all possible senses
            for sense_no, synset in enumerate(wn.synsets(tokens[0], nltkpos)):
                senses = set()
                sense_id = '%s-%d' % (word, sense_no+1)
                list_get(self.words2senses, word).append(sense_id)
                self.senses2concepts[sense_id] = synset.name
                
                # add class hierarchy to the database
                paths = []
                for s in synset.hypernym_paths():
                    path = [x.name for x in s]
                    senses.update(path)
                    paths.append(path)
                self.senses2hypernyms[sense_id] = paths
                for concept in senses:    
                    atomStr = 'is_a(%s,%s)' % (sense_id, concept)
                    FeatureExtractor.addEvidence(database, mln, atomStr)
            disjuncts = []
            for sense in self.words2senses[word]:
                disjuncts.append('has_sense(%s,%s)' % (word, sense))
            disjuncts.append('has_sense(%s,Nullsense)' % (word))
            disj = " v ".join(disjuncts)
            FeatureExtractor.addEvidence(database, mln, disj)

class MissingRoles(FeatureExtractor):
    '''
    Extracts possible word senses from WordNet given the part of speech
    of a word. Depends on the 'syntax' feature extractor.
    '''
    
    def __init__(self, mngr):
        FeatureExtractor.__init__(self, 'missingroles', mngr)

    def run(self, pracinference):
        coreDB = pracinference.databases['core']
        db = pracinference.databases['missingroles']
        mln = pracinference.mlns['missingroles']
        actioncore = pracinference.actioncore
        self.missingRoles = set(actioncore.action_roles)
        
        self.missingRoles.difference_update(['NULL'])
        specifiedRoles = set()
        for sol in coreDB.query('action_role(?w, ?r) ^ has_pos(?w, ?pos) ^ has_sense(?w, ?s) ^ !(?r = NULL)'):
            specifiedRoles.add(sol['?r'])
            db.addGroundAtom('has_pos(%s, %s)' % (sol['?w'], sol['?pos']))
            db.addGroundAtom('action_role(%s, %s)' % (sol['?w'], sol['?r']))
            db.addGroundAtom('has_pos(%s, %s)' % (sol['?w'], sol['?pos']))
            db.addGroundAtom('has_sense(%s, %s)' % (sol['?w'], sol['?s']))
        for sol in coreDB.query('has_sense(?w, ?s) ^ is_a(?s, ?c)'):
            db.addGroundAtom('is_a(%s,%s)' % (sol['?s'], sol['?c']))
#        for e, t in db.evidence.iteritems():
#            print e, t
        self.missingRoles.difference_update(specifiedRoles)
        
        db.domains['root'] = ['ROOT']
        
        # get all concepts that are known by the action core
        known_concepts = actioncore.known_concepts
        self.sense2concepts = {}
        for c in known_concepts:
            if c == 'NULL':
                continue
            synset = wn.synset(c)
            senses = set()
            sense_id = '%s' % (synset.name)
#            list_get(self.words2senses, word).append(sense_id)
#            self.senses2concepts[sense_id] = synset.name
            
            # add class hierarchy to the database
            for s in synset.hypernym_paths():
                senses.update([x.name for x in s])
            for concept in senses:
                atomStr = 'is_a(%s,%s)' % (sense_id, concept)
                FeatureExtractor.addEvidence(db, mln, atomStr)
            self.sense2concepts[c] = list(senses)
        # for each missing role, add a virtual word
        for role in self.missingRoles:
            word_id = '%s' % role
            FeatureExtractor.addEvidence(db, mln, 'action_role(%s, %s)' % (word_id, role))
            disj = ' v '.join(['has_sense(%s, %s)' % (word_id, s) for s in known_concepts])
            FeatureExtractor.addEvidence(db, mln, disj)
            
            
        

        
        
        
        
        
        
        
        
        
        
        
        
        
        
