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
import os
from prac.core.base import PRACModule, PRACPIPE
from pracmln.mln.base import parse_mln
from pracmln.mln.methods import LearningMethods
from prac.core.inference import PRACInferenceStep
# mapping from PennTreebank POS tags to NLTK POS Tags
from pracmln import Database, MLN, MLNQuery
from pracmln.mln.util import colorize, out, stop
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


log = logger(__name__)


class ActionCoreIdentification(PRACModule):
    
    def initialize(self):
        pass

    @PRACPIPE
    def __call__(self, pracinference, **params):
        log.debug('inference on %s' % self.name)
        
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RECOGNIZING ACTION CORES |', (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable ACTION CORE', (None, 'white', True), True)

        if params.get('project', None) is None:
            # load default project
            projectpath = self.project_path
            ac_project = MLNProject.open(projectpath)
        else:
            log.info(colorize('Loading Project from params', (None, 'cyan', True), True))
            projectpath = os.path.join(params.get('projectpath', None) or self.module_path, params.get('project').name)
            ac_project = params.get('project')

        dbs = pracinference.inference_steps[-1].output_dbs

        mlntext = ac_project.mlns.get(ac_project.queryconf['mln'], None)
        mln = parse_mln(mlntext, searchpaths=[self.module_path], projectpath=projectpath, logic=ac_project.queryconf.get('logic', 'FirstOrderLogic'), grammar=ac_project.queryconf.get('grammar', 'PRACGrammar'))
        known_concepts = mln.domains.get('concept', [])
        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')

        pngs = {}
        for db_ in dbs:
            db = wordnet_module.get_senses_and_similarities(db_, known_concepts)
            tmp_union_db = db.union(db_, mln=self.prac.mln)

            # result_db = list(kb.infer(tmp_union_db))[0]
            infer = MLNQuery(config=ac_project.queryconf, db=tmp_union_db, mln=mln).run()
            result_db = infer.resultdb

            unified_db = result_db.union(tmp_union_db, mln=self.prac.mln) # alternative to query below
            # only add inferred action_core atoms, leave out 0-evidence atoms
            # unified_db = tmp_union_db.copy(mln, mln=self.prac.mln)
            # for q in result_db.query('action_core(?w,?ac)'):
            #     log.info('Identified Action Core(s): {}'.format(colorize(q['?ac'], (None, 'white', True), True)))
            #     unified_db << 'action_core({},{})'.format(q['?w'],q['?ac'])
            
            #inf_step.output_dbs.append(unified_db)
            inf_step.output_dbs.extend(self.extract_multiple_action_cores(unified_db,wordnet_module,known_concepts))
            pngs[unified_db.domains.get('actioncore', [None])[0]] = get_cond_prob_png(ac_project.queryconf.get('queries', ''), dbs, filename=self.name)
            inf_step.png = pngs
            inf_step.applied_settings = ac_project.queryconf.config
        return inf_step
    
    def extract_multiple_action_cores(self, db,wordnet_module,known_concepts):
        dbs = []
        verb_list = []
        
        for q in db.query('action_core(?w,?ac)'):
            verb_list.append(q['?w'])
            
        if len(verb_list) < 2:
            return [db]

        # TODO improve the handling
        # Handle sentence with start with .....
        '''
        if len(verb_list) == 2:
            db.write()
            for word in ['start', 'Start']:
                for _ in db.query('prepc_with({}-1,?p)'.format(word)):
                    return [db]
        '''
        
        for verb in verb_list:
            db_ = Database(db.mln)
            processed_word_set = set()
            remaining_word_set = set()
            remaining_word_set.add(verb)

            while remaining_word_set:
                processed_word = remaining_word_set.pop()
                is_condition = False

                for atom, truth in sorted(db.evidence.iteritems()):
                    
                    _, pred, args = db.mln.logic.parse_literal(atom)
                    
                    if pred == "is_a" or pred == "has_sense":
                        continue
                    if pred == "action_core" and args[0] == processed_word:
                        db_ << (atom,truth)
                    elif len(args) == 1 and args[0] == processed_word:
                        db_ << (atom,truth)
                        is_condition = True
                    elif len(args) > 1:
                        word1 = args[0]
                        word2 = args[1]

                        dependency_word = ""
                        if word1 == processed_word:
                            dependency_word = word2
                        elif word2 == processed_word:
                            dependency_word = word1

                        if dependency_word and (
                                        dependency_word not in verb_list or
                                        pred == "event") and (
                                    dependency_word not in processed_word_set):
                            if pred != 'event' or not is_condition:
                                db_ << (atom,truth)
                            if pred != 'has_pos' and pred != 'event':
                                remaining_word_set.add(dependency_word)
                processed_word_set.add(processed_word)
            
        
            #Add valid senses and is_a concepts
            temp_sense_db = wordnet_module.get_senses_and_similarities(db_, known_concepts)
            valid_sense_list = temp_sense_db.domain('sense')
            valid_word_list = temp_sense_db.domain('word')
            
            for atom, truth in sorted(db.evidence.iteritems()):
                _, pred, args = db.mln.logic.parse_literal(atom)
                if pred != "is_a" and pred != "has_sense": continue
                
                if pred == "is_a" and args[0] in valid_sense_list:
                    db_ << (atom,truth)
                elif pred == "has_sense" and args[0] in valid_word_list and args[1] in valid_sense_list:
                    db_ << (atom,truth)
            dbs.append(db_)
        return dbs
    
    def train(self, praclearning):
        log = logger('prac')
        prac = praclearning.prac
        # get all the relevant training databases
        db_files = prac.getActionCoreTrainingDBs()
        nl_module = prac.getModuleByName('nl_parsing')
        syntactic_preds = nl_module.mln.predicates
        log.debug(db_files)
        dbs = filter(lambda x: type(x) is Database, map(lambda name: Database(self.mln, dbfile=name, ignore_unknown_preds=True), db_files))
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
        wordnet = prac.wordnet
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
                    new_db << ('is_a(%s,%s)' % (sense, known_concept), sim)
            training_dbs.append(new_db)

        log.info('Starting training with %d databases'.format(len(training_dbs)))
        # actioncore_KB = ActionCoreKB(self, 'action_cores')
        # actioncore_KB.wordnet = wordnet
        # actioncore_KB.train(training_dbs)
        # self.save_pracmt(actioncore_KB)
        
#
# class ActionCoreKB(PRACKnowledgeBase):
#     '''
#     Represents the probabilistic KB for learning and inferring
#     the correct action core.
#     '''
#
#     def train(self, training_dbs):
#         mln = self.module.mln
#         self.training_dbs = training_dbs
#         self.trained_mln = mln.learnWeights(training_dbs, LearningMethods.BPLL_CG, verbose=True, optimizer='bfgs')
        
        
        
        