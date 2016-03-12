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
import logging
from mln import readMLNFromFile, readDBFromFile#, MLNParsingError
from mln.methods import LearningMethods
import sys
from wcsp.converter import WCSPConverter
from mln.database import Database
import os
from prac.inference import PRACInferenceStep
from mln.util import mergeDomains
from utils import colorize
from pracutils.RolequeryHandler import RolequeryHandler
from prac.wordnet import WordNet
from sense_distribution import add_word_evidence_complete,\
    add_all_wordnet_similarities, get_prob_color
from mln.mln import readMLNFromString
from pracutils.pracgraphviz import render_gv
from pracutils.ActioncoreDescriptionHandler import ActioncoreDescriptionHandler
from ies_utils import MongoDatabaseHandler
import numpy



class SensesAndRoles(PRACModule):
    '''
    
    '''
    
    def initialize(self):
        pass
    
    def shutdown(self):
        pass
    
    
    def create_prob_distribution(self,missing_role,frame_list,roles_senses_dict,obj_list):
        harmonic_mean_list = []
        for obj in obj_list:
            roles_senses_dict_ = roles_senses_dict.copy()
            roles_senses_dict_[missing_role] = obj
            
            score_frame_matrix = numpy.array(map(lambda x: x.transform_to_frame_vector(roles_senses_dict_,{}),frame_list))
            argmax_index = score_frame_matrix.argmax()
            current_max_score = score_frame_matrix[argmax_index]
            harmonic_mean_list.append(current_max_score)
            '''
            print obj
            print current_max_score
            print frame_list[argmax_index].actioncore_roles[missing_role].nltk_wordnet_sense
            '''
            
    
        sum_harmonic_mean = sum(harmonic_mean_list)
        
        for i in range(0,len(obj_list)):
            print "{} : {}".format(obj_list[i],str(harmonic_mean_list[i]))
            
        
        raw_input("asd")
        
    def roleQueryBuilder(self, actioncore,predicate, domainList):
        query = predicate+'('
        
        if domainList[0].lower() == 'actioncore':
            query += actioncore
        else:
            query += "?"+domainList[0]
            
        if len(domainList) == 1:
            query += ")"
        else:
            i = 1
            for d in domainList[1:]:
                query += ","
                if d.lower() == 'actioncore':
                    query += actioncore
                else:
                    query += "?"+str(i)+d
            query += ")"
        return query
    
    def determine_missing_roles(self,db):
        db_ = db.duplicate()
        
        #Assuming there is only one action core
        for q in db.query('action_core(?w,?ac)'):
            actioncore = q['?ac']
            roles_db = RolequeryHandler.queryRoles(actioncore,db)
            
            inferred_roles_set = set()
            roles_dict = {}
            
            #Get inferred roles and the corresponding senses
            for atom, truth in sorted(roles_db.evidence.iteritems()):
                _ , predname, args = roles_db.mln.logic.parseLiteral(atom)
                if truth == 1.0:
                    inferred_roles_set.add(predname)
                    for sense_query in db.query('has_sense({},?s)'.format(args[0])):
                        roles_dict[sense_query['?s']] = predname
            
            #Build query, return only frames where all roles are defined
            actioncore_roles_list = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
            roles_query = [{"action_core" : "{}".format(actioncore)}]
            roles_query.extend(map(lambda x: {"actioncore_roles.{}".format(x) : {'$exists': 'true'}}, actioncore_roles_list))

            #Determine missing roles: All_Action_Roles\Inferred_Roles
            missing_role_set = set(actioncore_roles_list).difference(inferred_roles_set)
            
            #build query based on inferred senses and roles
            frame_result_list = MongoDatabaseHandler.get_frames_based_on_query({'$and' : roles_query})
            
            
            roles_senses_dict = RolequeryHandler.query_roles_and_senses_based_on_action_core(db_)
            self.create_prob_distribution(missing_role_set.pop(), frame_result_list, roles_senses_dict, ['oven.n.01','fridge.n.01','jar.n.01','coffee_maker.n.01','spoon.n.01','blender.n.01'])
            score_frame_matrix = numpy.array(map(lambda x: x.transform_to_frame_vector(roles_senses_dict,missing_role_set),frame_result_list))
            confidence_level = 0.7
            
            while frame_result_list and missing_role_set:
                argmax_index = score_frame_matrix.argmax()
                current_max_score = score_frame_matrix[argmax_index]
                
                if current_max_score < confidence_level:
                    break
                
                frame = frame_result_list[argmax_index]
                
                del frame_result_list[argmax_index]
                score_frame_matrix = numpy.delete(score_frame_matrix, argmax_index)
                
                missing_roles_contained_in_frame = missing_role_set.intersection(set(frame.actioncore_roles.keys()))
                
                for missing_role in missing_roles_contained_in_frame:
                    missing_role_sense = frame.actioncore_roles[missing_role]
                    print 'Found missing role "{}": {} Confidence: {}%'.format(missing_role,missing_role_sense.nltk_wordnet_sense,str(100*current_max_score))
                    raw_input("prompt")
                    atom_role = "{}({},{})".format(missing_role,missing_role_sense.word+"_mongo",actioncore)
                    db_.addGroundAtom(atom_role,1.0)
                    atom_sense = "{}({},{})".format('has_sense',missing_role_sense.word+"_mongo",missing_role_sense.nltk_wordnet_sense)
                    db_.addGroundAtom(atom_sense,1.0)
                    
                #missing_role_set = missing_role_set.difference(missing_roles_contained_in_frame)
            
            if missing_role_set:
                print "Cannot determine missing roles."
                raw_input("Enter")
                break
            
        return db_
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        
        #print colorize('+==========================================+', (None, 'green', True), True)
        #print colorize('| PRAC INFERENCE: RECOGNIZING %s ROLES  ' % ({True: 'MISSING', False: 'GIVEN'}[params.get('missing', False)]), (None, 'green', True), True)
        #print colorize('+==========================================+', (None, 'green', True), True)
        
        kb = params.get('kb', None)
        if kb is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
        else:
            kb = params['kb']
            dbs = kb.dbs
        self.kbs = []
        inf_step = PRACInferenceStep(pracinference, self)
        for db in dbs:
            db_ = db.duplicate()
#             db_.write(sys.stdout, color=True)
            for q in db.query('action_core(?w,?ac)'):
                actioncore = q['?ac']
                if actioncore == 'null': continue
                if kb is None:
                    #print 'Loading Markov Logic Network: %s' % colorize(actioncore, (None, 'white', True), True)
                    useKB = self.load_pracmt(actioncore)
                else:
                    useKB = kb
                self.kbs.append(useKB)
                params.update(useKB.query_params)
                unknown_roles = set()
                if 'missing' in params:
                    roles = useKB.query_mln.domains.get('role', [])
                    log.info('roles: %s' % roles)
                    specified_roles = []
                    for q in db.query('action_role(?w, ?r)'):
                        specified_roles.append(q['?r'])
                    unknown_roles = set(roles).difference(set(specified_roles))
                    log.info('unknown roles: %s' % unknown_roles)
                    if len(unknown_roles) > 0:
                        print colorize('DETECTED MISSING ACTION ROLES:', (None, 'red', True), True)
                    for i, role in enumerate(unknown_roles):
                        if role == 'null': continue
                        print colorize('  %s' % role, (None, 'red', True), True)
                        log.info('adding %s' % ('action_role(Skolem-%s, %s)' % (role, role)))
                        db_.addGroundAtom('action_role(Skolem-%s, %s)' % (role, role))
                #else:
                    #print 
                    # print 'Inferring given roles...'
                #print 
                concepts = useKB.query_mln.domains['concept']#mergeDomains(, self.merge_all_domains(pracinference))['concept']
                log.info('adding senses. concepts=%s' % concepts)
                wordnet_module = self.prac.getModuleByName('wn_senses')
                db_senses = wordnet_module.get_senses_and_similarities(db_, concepts)
                for atom, truth in sorted(db_senses.evidence.iteritems()):
                    if 'is_a' in atom or 'has_sense' in atom:
                        db_.addGroundAtom(atom,truth)  
                #db_.printEvidence()
                result_db_temp = list(useKB.infer(db_))
                result_db = []
                for r_db in result_db_temp:
                    if 'missing' not in params:
                        '''
                        for q in r_db.query('has_sense(?w, ?s)', truthThreshold=1):
                            #TODO Add additional formulas to avoid the using of null values
                            if q['?s'] == 'null': continue
                            print colorize('  WORD:', (None, 'white', True), True), q['?w'], 
                            print colorize('  SENSE:', (None, 'white', True), True), q['?s']
                            wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(r_db, q['?w']), q['?s'])
                            print
                        '''
                        #RolequeryHandler.queryRoles(actioncore,r_db).printEvidence()
                        
                        for atom, truth in sorted(db.evidence.iteritems()):
                            if 'is_a' in atom : continue
                            r_db.addGroundAtom(atom,truth)
                        
                        r_db = self.determine_missing_roles(r_db)
                            
                        result_db.append(r_db)
                '''        
                for ur in unknown_roles:
                    print '%s:' % colorize(ur, (None, 'red', True), True)
                    for q in r_db.query('action_role(?w, %s) ^ has_sense(?w, ?s)' % ur, truthThreshold=1):
                        self.prac.getModuleByName('wn_senses').printWordSenses(concepts, q['?s'])
                    print
                '''
                inf_step.output_dbs.extend(result_db)
        return inf_step
        
    
    def role_distributions(self, step):
        wn = WordNet()
        distrs = {}
        for db_ in step.output_dbs:
            db_.write(sys.stdout, color=True)
            for word in db_.domains['word']:
                for q in db_.query('action_core(?w,?ac)'):
                    actioncore = q['?ac']
                    kb = self.load_pracmt(actioncore)
                    mln = kb.query_mln
#                     mln.write(sys.stdout)
                    db = db_.duplicate(mln=mln)
                    for qs in db.query('!(EXIST ?w (has_sense(?w,?s)))'):
                        print 'removing', qs
                        db.removeDomainValue('sense', qs['?s']) 
                    print db.domains
                    for concept in db_.domains['concept']:
                        if concept not in mln.domains['concept']:
                            db.removeDomainValue('concept', concept)
                    db.write(sys.stdout, color=True)
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
                        db.write(sys.stdout)
                        result = mln.infer(queries='has_sense', method='EnumerationAsk', evidence_db=db, closedWorld=True, useMultiCPU=False)
                        g = wn.to_dot()
                        result.write(sys.stdout, color=True)
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
    
    
    
