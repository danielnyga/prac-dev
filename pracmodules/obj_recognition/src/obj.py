# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2013 by Daniel Nyga (nyga@cs.tum.edu)
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

from prac.core import PRACModule, PRACKnowledgeBase, PRACPIPE
import os
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from mln.methods import LearningMethods
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
import StringIO
import sys
from utils import colorize

   
class NLObjectRecognition(PRACModule):    

    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        log.debug('inference on %s' % self.name)
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| PRAC OBJECT RECOGNITION: RECOGNIZING OBJECTS|', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable ANNOTATION + simultaneous WORD SENSE DISMABIGUATION...', (None, 'white', True), True)
        
        if params.get('kb', None) is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
            kb = self.load_pracmt('default')
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        mln = kb.query_mln
        #known_concepts = mln.domains.get('concept', [])
        known_concepts = ['color.n.01','size.n.01','shape.n.01', 'part.n.01', 'peel.n.01', 'container.n.01']
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        for db in kb.dbs:
            db = wordnet_module.get_senses_and_similarities(db, known_concepts)
            db.write(sys.stdout, color=True)
            print '---'
            result_db = list(kb.infer(db))
            inf_step.output_dbs.extend(result_db)
            print
            for r_db in result_db:
                for q in r_db.query('color(?w, ?color)'):
                    print q['?color']
                    if q['?color'] == 'null': continue
                    print 'Identified color(s):', colorize(q['?color'], (None, 'white', True), True)
                print

                for q in r_db.query('size(?w, ?size)'):
                    print q['?size']
                    if q['?size'] == 'null': continue
                    print 'Identified size(s):', colorize(q['?size'], (None, 'white', True), True)
                print

                for q in r_db.query('shape(?w, ?shape)'):
                    print q['?shape']
                    if q['?shape'] == 'null': continue
                    print 'Identified shape(s):', colorize(q['?shape'], (None, 'white', True), True)
                print

                for q in r_db.query('relation(?w, isA, ?concept)'):
                    print q['?concept']
                    if q['?concept'] == 'null': continue
                    print 'Identified relation(s):', colorize(q['?concept'], (None, 'white', True), True)
                print

                for q in r_db.query('relation(?w, hasA, ?concept)'):
                    print q['?concept']
                    if q['?concept'] == 'null': continue
                    print 'Identified relation(s):', colorize(q['?concept'], (None, 'white', True), True)
                print

                print 'Inferred most probable word senses:'
                for q in r_db.query('has_sense(?w, ?s)'):
                    if q['?s'] == 'null': continue
                    print '%s:' % q['?w']
                    wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(r_db, q['?w']), q['?s'])
                    print 
                    
        return inf_step
    
    
    def train(self, praclearning):
        log = logging.getLogger('prac')
        prac = praclearning.prac
        # get all the relevant training databases
        db_files = prac.getActionCoreTrainingDBs()
        nl_module = prac.getModuleByName('nl_parsing')
        syntactic_preds = nl_module.mln.predicates
        log.debug(db_files)
        dbs = filter(lambda x: type(x) is Database, map(lambda name: readDBFromFile(self.mln, name, True), db_files))
        log.debug(dbs)
        new_dbs = []
        training_dbs = []
        known_concepts = []
        log.debug(self.mln.domains)
        for db in dbs:
            if not 'actioncore' in db.domains: continue
            if not 'concept' in db.domains: continue
            for c in db.domains['concept']:
                known_concepts.append(c)
            new_dbs.append(db)
        wordnet = WordNet()
        for db in new_dbs:
            new_db = db.duplicate()
            for sol in db.query('has_sense(?w, ?s) ^ is_a(?s, ?c)'):
                word = sol['?w']
                sense = sol['?s']
                concept = sol['?c']
                synset = wordnet.synset(concept)
                for known_concept in known_concepts:
                    known_synset = wordnet.synset(known_concept)
                    if known_synset is None or synset is None: sim = 0
                    else: sim = wordnet.wup_similarity(synset, known_synset)
                    new_db.addGroundAtom('is_a(%s,%s)' % (sense, known_concept), sim)
            training_dbs.append(new_db)
        log.info('Starting training with %d databases' % len(training_dbs))
        actioncore_KB = ActionCoreKB(self, 'action_cores')
        actioncore_KB.wordnet = wordnet
        actioncore_KB.train(training_dbs)
        self.save_pracmt(actioncore_KB)
        
        
class ActionCoreKB(PRACKnowledgeBase):
    '''
    Represents the probabilistic KB for learning and inferring
    the correct action core.
    '''
    
    def train(self, training_dbs):
        mln = self.module.mln
        self.training_dbs = training_dbs
        self.trained_mln = mln.learnWeights(training_dbs, LearningMethods.BPLL_CG, verbose=True, optimizer='bfgs')
        
        
        
