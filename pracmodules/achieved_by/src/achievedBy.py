# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2015 by Daniel Nyga (nyga@cs.tum.edu)
# (C) 2015 by Sebastian Koralewski (seba@informatik.uni-bremen.de)
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
from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from prac.core.wordnet import WordNet
from pracmln import Database
from pracmln.mln.util import colorize
from pracmln.praclog import logger

log = logger(__name__)

class AchievedBy(PRACModule):
    '''

    '''

    def initialize(self):
        pass

    def shutdown(self):
        pass

    def extendDBWithAchievedByEvidence(self, db, querymln):
        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
        acdomain = db.mln.domains.get("actioncore")
        acdomain.extend(db.domains.get("actioncore"))
        acdomain = set(acdomain)
        db_ = Database(querymln)

        for ac1 in acdomain:
            for ac2 in acdomain:
                if ac1 == actioncore: continue
                db_ << ("achieved_by({},{})".format(ac1,ac2),0)

        for atom, truth in sorted(db.evidence.iteritems()):
            db_ << (atom,truth)

        return db_

    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RECOGNIZING ACHIEVED BY  ' , (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)
        
        kb = params.get('kb', None)
        if kb is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
            dbs = kb.dbs
        
        inf_step = PRACInferenceStep(pracinference, self)
        
        for db in dbs:
            
            for q in db.query('action_core(?w,?ac)'):
                actioncore = q['?ac']
                #This list is used to avoid an infinite loop during the achieved by inference.
                #To avoid this infinite loop, the list contains the pracmlns which were inferenced during the process.
                #Every pracmln should be used only once during the process because the evidence for the inference will always remain the same.
                #So if the pracmln hadnt inferenced a plan in the first time, it will never do it.

                db_ = Database(db.mln)
                #Need to remove possible achieved_by predicates from previous achieved_by inferences
                for atom, truth in sorted(db.evidence.iteritems()):
                    if 'achieved_by' in atom: continue
                    db_ << (atom,truth)

                if kb is None:
                    log.info('Loading Markov Logic Network: %s' % colorize(actioncore, (None, 'cyan', True), True))
                    kb = self.load_prac_kb(actioncore)
                    
                concepts = kb.query_mln.domains.get('concept', [])
                wordnet_module = self.prac.getModuleByName('wn_senses')
                db = wordnet_module.get_senses_and_similarities(db_, concepts)

                unified_db = db.union(kb.query_mln, db_)

                #Inference achieved_by predicate        
                db_ = self.extendDBWithAchievedByEvidence(unified_db,kb.query_mln)

                log.info('using following db for achieved_by inference')
                db_.write(bars=False)
                result_db = list(kb.infer(db_))[0]

                # unified_db = result_db.union(kb.query_mln, db_)
                # only add inferred achieved_by atoms, leave out 0-evidence atoms
                for q in result_db.query('achieved_by(?ac1,?ac2)'):
                    unified_db << 'achieved_by({},{})'.format(q['?ac1'],q['?ac2'])

                inf_step.output_dbs.append(unified_db)

        if kb is not None:
            png, ratio = kb.get_cond_prob_png(filename=self.name)
            inf_step.png = (png, ratio)
            inf_step.applied_kb = kb.filename 
        return inf_step
    
