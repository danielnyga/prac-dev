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

from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from prac.core.wordnet import WordNet
from prac.pracutils.ActioncoreDescriptionHandler import \
    ActioncoreDescriptionHandler
from prac.pracutils.pracgraphviz import render_gv
from prac.sense_distribution import add_all_wordnet_similarities, \
    get_prob_color
from pracmln import MLNQuery, Database
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, out
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from webmln.gui.pages.utils import get_cond_prob_png


class SensesAndRoles(PRACModule):
    '''
    
    '''
    
    def initialize(self):
        pass
    
    def shutdown(self):
        pass
    
    def roleQueryBuilder(self, actioncore, predicate, domainList):
        # assuming that the role predicates are always of the form
        # predname(?x, actioncore)
        return '{}(?{},{})'.format(predicate, domainList[0], actioncore)

    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logger(self.name)
        
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RECOGNIZING %s ROLES     ' % ({True: 'MISSING', False: 'GIVEN'}[params.get('missing', False)]), (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)

        dbs = pracinference.inference_steps[-1].output_dbs
        inf_step = PRACInferenceStep(pracinference, self)

        for olddb in dbs:
            db_copy = olddb.copy(mln=self.prac.mln)
            for q in olddb.query('action_core(?w,?ac)'):
                actioncore = q['?ac']
                if actioncore == 'null': continue

                if params.get('project', None) is None:
                    log.info('Loading Project: %s.pracmln' % colorize(actioncore, (None, 'cyan', True), True))
                    projectpath = os.path.join(self.module_path, '{}.pracmln'.format(actioncore))
                    project = MLNProject.open(projectpath)
                else:
                    log.info(colorize('Loading Project from params', (None, 'cyan', True), True))
                    projectpath = os.path.join(params.get('projectpath', None) or self.module_path, params.get('project').name)
                    project = params.get('project')

                mlntext = project.mlns.get(project.queryconf['mln'], None)
                mln = parse_mln(mlntext, searchpaths=[self.module_path], projectpath=projectpath, logic=project.queryconf.get('logic', 'FirstOrderLogic'), grammar=project.queryconf.get('grammar', 'PRACGrammar'))
                known_concepts = mln.domains.get('concept', [])
                wordnet_module = self.prac.getModuleByName('wn_senses')

                unknown_roles = set()
                if 'missing' in params:
                    roles = mln.domains.get('role', [])
                    log.info('roles: %s' % roles)
                    specified_roles = []
                    for q in olddb.query('action_role(?w, ?r)'):
                        specified_roles.append(q['?r'])
                    unknown_roles = set(roles).difference(set(specified_roles))
                    log.info('unknown roles: %s' % unknown_roles)
                    if len(unknown_roles) > 0:
                        log.info(colorize('DETECTED MISSING ACTION ROLES:', (None, 'red', True), True))
                    for i, role in enumerate(unknown_roles):
                        if role == 'null': continue
                        log.info(colorize('  %s' % role, (None, 'red', True), True))
                        log.info('adding %s' % ('action_role(Skolem-%s, %s)' % (role, role)))
                        db_copy << ('action_role(Skolem-%s, %s)' % (role, role))
                else:
                    log.info('Inferring given roles...')

                # adding senses and similarities. might be obsolete as it has
                # already been performed in ac recognition
                log.info('adding senses. concepts=%s' % known_concepts)
                db = wordnet_module.get_senses_and_similarities(db_copy, known_concepts)

                # we need senses and similarities as well as original evidence
                tmp_union_db = db.union(db_copy, mln=self.prac.mln)

                # ignore roles of false ac's
                new_tmp_union_db = tmp_union_db.copy(mln=self.prac.mln)
                roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
                for q in tmp_union_db.query('action_core(?w, ?ac)', thr=0):
                    ac = q['?ac']
                    w = q['?w']
                    if ac == actioncore:
                        for r1 in roles:
                            # words with sense null can be discarded as they can not have a role
                            for q2 in tmp_union_db.query('has_sense(?w, null)', thr=1):
                                new_tmp_union_db << ('{}({},{})'.format(r1, q2['?w'], ac), 0)
                        continue
                    for r in roles:
                        new_tmp_union_db << ('{}({},{})'.format(r, w, ac), 0)

                infer = MLNQuery(config=project.queryconf, db=new_tmp_union_db, mln=mln).run()
                result_db = infer.resultdb

                # get query roles for given actioncore and add inference results
                # for them to final output db. ignore 0-truth results.
                unified_db = new_tmp_union_db.union(result_db, mln=self.prac.mln)

                out('evidence db')
                unified_db.write(bars=False)


                # argdoms = kb.query_mln.predicate(role).argdoms
                roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
                out('roles:', roles)
                new_result = Database(self.prac.mln)
                for atom, truth in unified_db.evidence.iteritems():
                    if any(r in atom for r in roles):
                        (_, predname, args) = self.prac.mln.logic.parse_literal(atom)
                        if not args[-1] == actioncore:
                            out('omitted role ', r, 'with actioncore', args[-1], atom)
                            continue
                        out('adding', atom, truth)
                    new_result << (atom, truth)
                new_result.write(bars=False)

                    # for q in result_db.query('{}(?{})'.format(role, ',?'.join(argdoms))):
                    #     log.info('Adding {}({}) to output database'.format(role, ','.join([q['?{}'.format(x)] for x in argdoms])))
                    #     unified_db << '{}({})'.format(role, ','.join([q['?{}'.format(x)] for x in argdoms]))
                    # add inferred senses to final output db

                if 'missing' not in params:
                    for q in unified_db.query('has_sense(?w, ?s)', thr=1):
                        #TODO Add additional formulas to avoid the using of null values
                        if q['?s'] == 'null': continue
                        print colorize('  WORD:', (None, 'white', True), True), q['?w']
                        print colorize('  SENSE:', (None, 'white', True), True), q['?s']
                        wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(unified_db, q['?w']), q['?s'])
                        print

                for ur in unknown_roles:
                    print '%s:' % colorize(ur, (None, 'red', True), True)
                    for q in unified_db.query('action_role(?w, %s) ^ has_sense(?w, ?s)' % ur, thr=1):
                        self.prac.getModuleByName('wn_senses').printWordSenses(known_concepts, q['?s'])
                    print

                inf_step.output_dbs.append(new_result)

            png, ratio = get_cond_prob_png(project.queryconf.get('queries', ''), dbs, filename=self.name)
            inf_step.png = (png, ratio)
            inf_step.applied_settings = project.queryconf.config
        return inf_step
        
    
    def role_distributions(self, step):
        wn = WordNet()
        distrs = {}
        for db_ in step.output_dbs:
            for word in db_.domains['word']:
                for q in db_.query('action_core(?w,?ac)'):
                    actioncore = q['?ac']
                    kb = self.load_prac_kb(actioncore)
                    mln = kb.query_mln
                    db = db_.copy(mln=mln)
                    for qs in db.query('!(EXIST ?w (has_sense(?w,?s)))'):
                        print 'removing', qs
                        db.removeDomainValue('sense', qs['?s']) 
                    print db.domains
                    for concept in db_.domains['concept']:
                        if concept not in mln.domains['concept']:
                            db.removeDomainValue('concept', concept)
                    for res in db_.query('has_sense(%s, ?s)' % (word)):
                        sense = res['?s']
                        if sense == 'null': continue
                        roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
                        role = None
                        for r in roles:
                            vars = ['?v%d' % i for i in range(len(db_.mln.predicates[r])-1)]
                            br = False
                            for qr in db_.query('%s(%s,%s)' % (r, ','.join(vars), actioncore)): 
                                for v in vars:
                                    if qr[v] == word: 
                                        role = r
                                        br = True
                                        break
                                if br: break
                            if br: break
                        print role
                        if role is None: continue
                        db.retractGndAtom('has_sense(%s, %s)' % (word, sense))
#                         wnmod = self.prac.getModuleByName('wn_senses')
#                         for synset in wnmod.get_possible_meanings_of_word(db, word):
#                             db.removeDomainValue('sense', synset.name)
                        add_all_wordnet_similarities(db, wn)
                        result = mln.infer(queries='has_sense', method='EnumerationAsk', evidence_db=db, closedWorld=True, useMultiCPU=False)
                        g = wn.to_dot()
                        maxprob = 0.
                        for atom, truth in result.evidence.iteritems():
                            _, predname, args = db.mln.logic.parseLiteral(atom)
                            concept = args[1]
                            if predname == 'has_sense' and args[0] == word and args[1] != 'null':
                                maxprob = max(maxprob, truth) 

                        for atom, truth in result.evidence.iteritems():
                            _, predname, args = db.mln.logic.parseLiteral(atom)
                            concept = args[1]
                            if predname == 'has_sense' and args[0] == word and args[1] != 'null':
                                print concept, get_prob_color(truth)
                                if concept in wn.known_concepts:
                                    g.node(concept, fillcolor=get_prob_color(truth / maxprob))
                        render_gv(g, 'prac-%s-%s.svg' % (actioncore, role))
                        distrs[role] = render_gv(g)
        return distrs
    
    
    
