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
import os
from nltk.corpus import wordnet as wn

class PRACLearner(object):
    
    def __init__(self, actioncore):
        self.actioncore = actioncore
        self.loadDatabases()
        
    def loadDatabases(self):
        actioncore = self.actioncore
        self.dbs = actioncore.mln.loadPRACDatabases(os.path.join('models', actioncore.name, 'db'))
        actioncore.known_concepts = set()
        for d in self.dbs:
            actioncore.known_concepts.update(d.domains['sense'])
            # load the super-concepts into the database
            for s in d.query('is_a(?s, ?c)'):
                superconcepts = set()
                if s['?c'] == 'NULL': continue
                for path in wn.synset(s['?c']).hypernym_paths():
                    superconcepts.update(map(lambda x: x.name, path))
                superconcepts.difference_update(actioncore.known_concepts)
                for c in superconcepts:
                    d.addGroundAtom('is_a(%s,%s)' % (s['?s'], c))
        print actioncore.known_concepts
                
    def run(self):
        self.actioncore.learnedMLN = self.actioncore.mln.duplicate()
        for f_templ in self.actioncore.formula_templates:
            self.actioncore.learnedMLN.addFormula(f_templ, hard=f_templ.isHard)
        self.actioncore.learnedMLN.learnWeights(self.dbs)
    