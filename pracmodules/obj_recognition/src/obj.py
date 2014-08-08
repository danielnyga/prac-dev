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

from prac.core import PRACModule, PRACKnowledgeBase, PRACPIPE, DescriptionKnowledgeBase
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from mln.methods import LearningMethods
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
import sys, os
from utils import colorize

class NLObjectRecognition(PRACModule):    

    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        log.info('Running {}'.format(self.name))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| PRAC OBJECT RECOGNITION: RECOGNIZING OBJECTS|', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable object based on nl description properties...', (None, 'white', True), True)
        
        if params.get('dkb') is not None:
            dkb = params.get('dkb')
        else:
            dkb = self.load_dkb('fruit')
        print 'Using DKB: {}'.format(colorize(dkb.name, (None, 'yellow', True), True))
        dkb.kbmln.write(sys.stdout, color=True) # todo remove, debugging only
        print 

        if params.get('kb', None) is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
            kb = self.load_pracmt('default')
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        mln = dkb.kbmln

        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        
        result_dbs = []
        # process databases
        for db in kb.dbs:
            # adding evidence properties to new query db
            res_db = Database(mln)
            words = []
            for res in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                if res['?prop'] == 'null': continue
                if res['?sense'] == 'null': continue
                words.append(res['?sense'])
                atom = 'property({}, {}, {})'.format(res['?cluster'], res['?sense'], res['?prop'])
                res_db.addGroundAtom(atom)

            # adding word similarities
            words += mln.domains.get('word', []) # + words from database
            res_db = wordnet_module.add_senses_and_similiarities_for_words(res_db, words)
            
            # infer and update output dbs
            print kb.query_params
            inferred_db = mln.infer(evidence_db=res_db, **kb.query_params)
            # print colorize('Inferred DB...', (None, 'green', True), True) 
            # inferred_db.write(sys.stdout,color=True)
            inf_step.output_dbs.extend([inferred_db])

            for r_db in res_db.query('object(?cluster, ?concept)'):
                # print annotations found in result db
                if q['?concept'] == 'null': continue
                print 'object({}, {})'.format(q['?cluster'], colorize(q['?concept'], (None, 'white', True), True))
        return inf_step


    def train(self, praclearning):
        print colorize('+===================+', (None, 'green', True), True)
        print colorize('| No training used  |', (None, 'green', True), True)
        print colorize('+===================+', (None, 'green', True), True)
