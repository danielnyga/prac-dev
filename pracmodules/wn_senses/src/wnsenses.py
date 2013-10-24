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

from prac.core import PRACModule, PRACPIPE
from nltk.corpus import wordnet as wn
import copy
import logging
from pracutils import list_get
from actioncore.inference import PRACInferenceStep
from mln.MarkovLogicNetwork import readMLNFromFile
import os
from mln.database import Database

# mapping from PennTreebank POS tags to NLTK POS Tags
nounTags = ['NN', 'NNS', 'NNP']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
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
    
#     def __init__(self, prac):
#         PRACModule.__init__(self, prac)

    def initialized(self):
        self.decls_mln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'decls.mln'))
    
    @PRACPIPE
    def run(self, pracinference):
        log = logging.getLogger('WNSenses')
        inf_step = PRACInferenceStep(pracinference, self)
        
        for db in pracinference.module2infSteps['nl_parsing'][0].output_dbs:
            database = Database(self.decls_mln)
            for gndLit in db.iterGroundLiteralStrings():
                database.addGroundAtom(gndLit)
                
            # default sense is the NULL-sense
            database.addGroundAtom('is_a(Nullsense, NULL)')
            # map from word constants to possible senses
            self.words2senses = {}
            # map from sense constants to taxonomic concepts
            self.senses2concepts = {}
            # map from sense constants to hypernym paths
            self.senses2hypernyms = {}
            
            for word in database.domains['word']:
#                 disjuncts = []
                tokens = word.rsplit('-', 1)
                
                postag = None
                for s in database.query('has_pos(%s,?pos)' % word):
                    postag = s['?pos']
                if postag is None:
                    log.exception('No POS tag found for word "%s"' % word)
                nltkpos = posMap.get(postag, None)
                if nltkpos is None:
                    database.addGroundAtom('has_sense(%s,Nullsense)' % word)
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
                        database.addGroundAtom(atomStr)
            # add evidence for the inapplicable senses
            for word in database.domains['word']:
                for sense in self.senses2concepts:
                    if not sense in self.words2senses.get(word, []):
                        database.addGroundAtom('!has_sense(%s,%s)' % (word, sense))
            inf_step.output_dbs.append(database)
        return inf_step
#                 disjuncts = []
#                 for sense in self.words2senses[word]:
#                     disjuncts.append('has_sense(%s,%s)' % (word, sense))
#                 disjuncts.append('has_sense(%s,Nullsense)' % (word))
#                 disj = " v ".join(disjuncts)
#                 FeatureExtractor.addEvidence(database, mln, disj)
     