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

from prac.core import PRACModule, PRACPIPE, PRACKnowledgeBase
from actioncore.inference import PRACInferenceStep
import logging
from mln import readMLNFromFile, readDBFromFile#, MLNParsingError
from mln.methods import ParameterLearningMeasures
import sys
from wcsp.converter import WCSPConverter
from mln.database import Database
import os


class SensesAndRoles(PRACModule):
    '''
    
    '''
    
    stanford_parser = None
    
    def initialize(self):
        # load the action core-specific MLNs
        self.mlns = {}
        log = logging.getLogger(self.name)
        for action_core in self.prac.action_cores:
            try:
                mln = readMLNFromFile(os.path.join(self.module_path, 'mln', '%s.mln' % action_core), grammar='PRACGrammar', logic='FuzzyLogic')
                log.info(mln.predicates)
                self.mlns[action_core] = mln
            except Exception, e:# Exception:#MLNParsingError:
                log.error(e.message)
                log.error('No MLN found for "%s".' % action_core)

    def shutdown(self):
        pass
    
    @PRACPIPE
    def infer(self, pracinference):
        log = logging.getLogger(self.name)
        inf_step = PRACInferenceStep(pracinference, self)
        for input_db in pracinference.module2infSteps['ac_recognition'][0].output_dbs:
            for s in input_db.query('action_core(?ac)'):
                action_core_name = s['?ac']
            if action_core_name is None:
                log.exception('Action core cannot be found. Run the ac_recognition module first.')
            log.info('Loading microtheory for %s' % action_core_name)
            microtheory = self.load_pracmt(action_core_name)
            mln = microtheory.learned_mln
            query_db = input_db.duplicate(mln, ignoreUnknownPredicates=True)
            log.info(mln.domains['concept'])
            # filter out unknown concepts
            log.info(mln.domains)
            for _, lit in input_db.iterGroundLiteralStrings('is_a'):
                _, _, params = mln.logic.parseLiteral(lit)
                concept = params[1]
                if not concept in mln.domains['concept']:
                    query_db.retractGndAtom(lit)
            log.info('database evidence')
            query_db.printEvidence()
            action_core_name = None
            mln.setClosedWorldPred(*[p for p in microtheory.learned_mln.predicates if p not in ('has_sense', 'action_role')])
            mln.write(sys.stdout, color=True)
            mrf = mln.groundMRF(query_db)
            converter = WCSPConverter(mrf)
            result_db = converter.getMostProbableWorldDB()
            for res in result_db.query('has_sense(?w,?s) ^ action_role(?w,?r)'):
                print res
        return inf_step
                
    @PRACPIPE
    def train(self, praclearn):
        log = logging.getLogger('PRAC')
        prac = self.prac
        mts = []
        if praclearn.microtheories is None:
            mts = prac.action_cores
        else:
            mts = praclearn.microtheories
        wn_senses = prac.getModuleByName('wn_senses')
        for mt in mts:
            db_files = prac.getActionCoreTrainingDBs(mt)
            # add the fuzzy class hierarchy evidences
            training_dbs = []
            mln = self.mlns[mt]
            for db in db_files: 
                db = readDBFromFile(mln, db)
                training_dbs.append(db)
            wn_senses.addFuzzyEvidenceToDBs(*training_dbs)
            learnedMT = SensesAndRolesMT(self, mt)
            learnedMT.train(training_dbs)
            self.save_pracmt(learnedMT)
            
    
class SensesAndRolesMT(PRACKnowledgeBase):
    
    def train(self, training_dbs):
        log = logging.getLogger('PRAC')
        mln = self.module.mlns[self.name]
        self.training_dbs = training_dbs
        mln.write(sys.stdout, color=True)
        self.learned_mln = mln.learnWeights(training_dbs, ParameterLearningMeasures.BPLL_CG, optimizer='bfgs', verbose=True)#, gaussianPriorSigma=1.)
        
        
        
    
    
    
