from prac.core import PRACModule, PRACKnowledgeBase, PRACPIPE
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
from utils import colorize

#How to call it 
#actionCore = prac.getModuleByName('ac_recognition')
#prac.run(infer,actionCore,kb=actionCore.load_pracmt('robohow'))

class PRACWSD(PRACModule):
    def initialize(self):
      pass

    @PRACPIPE
    def __call__(self,pracinference, **params):
            log = logging.getLogger(self.name)
            log.debug('inference on %s' % self.name)
 
            print colorize('+==========================================+', (None, 'green', True), True)
            print colorize('| PRAC INFERENCE: WORD SENSE DISMABIGUATION|', (None, 'green', True), True)
            print colorize('+==========================================+', (None, 'green', True), True)
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
            
            mln = kb.query_mln
            logic = kb.query_params['logic']
            fol =  False
            if(logic == 'FirstOrderLogic'):
                fol = True
            known_concepts = mln.domains.get('concept', [])
            inf_step = PRACInferenceStep(pracinference, self)
            wordnet_module = self.prac.getModuleByName('wn_senses')
            
            
            for db in kb.dbs:
                db = wordnet_module.get_senses_and_similarities(db, known_concepts)
                result_db = list(kb.infer(db))
                inf_step.output_dbs.extend(result_db)
                print
                for r_db in result_db:
                    print 'Inferred most probable word senses:'
                    for q in r_db.query('has_sense(?w, ?s)'):
                        if q['?s'] == 'null': continue
                        print '%s:' % q['?w']
                        #wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(r_db, q['?w']), q['?s'])
                        print

            return inf_step
            
