# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2013 by Mareike Picklum (mareikep@cs.tum.edu)
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
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from mln.methods import LearningMethods
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
import sys
from utils import colorize

class NLObjectRecognition(PRACModule):    

    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        log.info('Running %s' % self.name)
        
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

        known_concepts = mln.domains.get('concept', [])
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        for db in kb.dbs:
            db = wordnet_module.get_senses_and_similarities(db, known_concepts)
            db.write(sys.stdout, color=True)

            if 'cluster' in db.domains:
                domains = db.domains['cluster']
                domains.append('cluster')
            else:
                db.domains['cluster'] = ['cluster']

            result_db = list(kb.infer(db))
            print colorize('+============Result DB=================+', (None, 'green', True), True)
            result_db[0].write(sys.stdout, color=True)
            inf_step.output_dbs.extend(result_db)

            for r_db in result_db:
                for annot in ['is_a', 'has_a', 'color', 'shape', 'size', 'object']:
                    for q in r_db.query('{}(?o, ?c)'.format(annot)):
                        if q['?c'] == 'null': continue
                        print '{}({},{})'.format(colorize(annot, (None, 'yellow', True), True)  , q['?o'], colorize(q['?c'], (None, 'white', True), True))
                    print
                print 'Inferred most probable word senses:'
                for q in r_db.query('has_sense(?w, ?s)'):
                    if q['?s'] == 'null': continue
                    print '%s:' % q['?w']
                    wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(r_db, q['?w']), q['?s'])
                    
        return inf_step


    def train(self, praclearning):
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| DnD... learning in progress...              |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)

        log = logging.getLogger('prac')
        prac = praclearning.prac
        # get all the relevant training databases
        db_files = prac.getActionCoreTrainingDBs()
        nl_module = prac.getModuleByName('nl_parsing')
        syntactic_preds = nl_module.mln.predicates
        log.debug(db_files)
        kb = self.load_pracmt('obj_recog')
        mln = kb.query_mln
        mln.printFormulas()
        inputfile = '/win/common/Uni/semester10_sose14/prac/pracmodules/obj_recognition/mln/ts_stanford_wn_man.db'
        dbs = readDBFromFile(mln, inputfile, True)

        learnedMLN = mln.learnWeights([inputfile], LearningMethods.BPLL_CG, verbose=True, optimizer='bfgs',maxSteps=100,debug='debug',learningRate=1)
        learnedMLN.printFormulas() 
        
#         new_dbs = []
#         training_dbs = []
#         known_concepts = []
#         log.info(mln.domains)
#         for db in dbs:
#             if not 'concept' in db.domains: continue
#             if not 'cluster' in db.domains: continue
#             for c in db.domains['concept']:
#                 known_concepts.append(c)
#             new_dbs.append(db)
#         wordnet = WordNet()
#         for db in new_dbs:
#             new_db = db.duplicate()
# 
#             # begin adding ground atoms...
#             for sol in db.query('is_a(?sense,?concept)'):
#                 sense = sol['?sense']
#                 concept = sol['?concept']
#                 sense = sol['?sense'] # TODO! 
#                 synset = wordnet.synset(concept)
#                 for known_concept in known_concepts:
#                     known_synset = wordnet.synset(known_concept)
#                     if known_synset is None or synset is None: sim = 0
#                     else: sim = wordnet.similarity(synset, known_synset)
#                     new_db.addGroundAtom('is_a(%s,%s)' % (sense, concept), sim)
# 
# 
#             annotations = ['size', 'shape', 'color', 'has_a']
#             for annot in annotations:
#                 for sol in db.query('has_sense(?word,?concept) ^ {}(?object,?concept)'.format(annot)):
#                     object_ = sol['?object']
#                     sense = sol['?concept']
#                     word = sol['?word']
#                     synset = wordnet.synset(sense)
#                     for known_concept in known_concepts:
#                         known_synset = wordnet.synset(known_concept)
#                         if known_synset is None or synset is None: sim = 0
#                         else: sim = wordnet.similarity(synset, known_synset)
#                         new_db.addGroundAtom('{}({},{})'.format(annot, object_, sense), sim)
# 
#             # finish adding ground atoms...
#             #new_db.write(sys.stdout, color=True)
#             training_dbs.append(new_db)
#         log.info('Starting training with %d databases' % len(training_dbs))
# 
#         # actioncore_KB = ActionCoreKB(self, 'action_cores')
#         # actioncore_KB.wordnet = wordnet
#         training_dbs[0].write(sys.stdout, color=True)
# #         trained_mln = mln.learnWeights(training_dbs, LearningMethods.PLL, verbose=True, optimizer='bfgs')
#         #trained_mln = mln.learnWeights(training_dbs, LearningMethods.BPLL_CG, verbose=True, optimizer='bfgs',maxSteps=100,debug='debug',learningRate=1)
#         #trained_mln = mln.learnWeights(training_dbs, LearningMethods.BPLL, verbose=True, optimizer='bfgs')
#         trained_mln.printFormulas() 
#         outputfile = '/win/common/Uni/semester10_sose14/prac/pracmodules/obj_recognition/mln/wts_pll_ts_stanford_wn_man.db'
#         trained_mln.write(file(outputfile, "w"))
#         # self.save_pracmt(actioncore_KB)
#         #verbose=False,debug='ERROR'


        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| Learnt:                                     |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        