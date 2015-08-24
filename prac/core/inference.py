# PROBABILISTIC ROBOT ACTION CORES - INFERENCE
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

# from actioncore.features import FeatureManager, Syntax, WordSenses, MissingRoles
from prac.pracutils import StopWatch
# from actioncore import PRAC, PRACReasoner, PRACPIPE
# from nltk.corpus import wordnet as wn
# from linguistics.parsing import Parser
#from linguistics.verbalizer import PRACVerbalizer
#from linguistics import NLISentence
# from linguistics import HRIDialog

import os
import re
import math
import operator
from graphviz.dot import Digraph
from prac.pracutils.ActioncoreDescriptionHandler import ActioncoreDescriptionHandler
import sys
from prac.pracutils.pracgraphviz import render_gv
from pracmln import Database


class PRACInferenceStep(object):
    '''
    Wrapper class encapsulating a single inference step in the PRAC
    pipeline. It consists of n input databases and m output databases.
    '''
    
    def __init__(self, pracinfer, module, input_dbs=None):
        '''
        Initializes the inference step.
        - pracinfer:    reference to the PRACInference object.
        - module:       reference to the PRACModule performing this inference step.
        - input_dbs:    list of databases taken as inputs.
        '''
        if input_dbs is None:
            self.input_dbs = []
        else:
            self.input_dbs = input_dbs
        self.module = module
        self.prac = pracinfer.prac
        self.pracinference = pracinfer
        self.output_dbs = []
        self.watch = StopWatch()

class PRACInference(object):
    '''
    Represents an inference chain in PRAC:
    - prac:            reference to the PRAC instance.
    - instructions:    list of natural-language sentences subject to inference.
    '''
    
    def __init__(self, prac, instructions):
        self.prac = prac
        prac.uninitAllModules()
        self.instructions = instructions
        self.inference_steps = []
        self.watch = StopWatch()
    
    def next_module(self):
        if not self.inference_steps:
            return 'nl_parsing'
        previous_module = self.inference_steps[-1].module.name 
         
        if previous_module == 'nl_parsing':
            return 'ac_recognition'
        elif previous_module == 'ac_recognition':
            return 'senses_and_roles'
        elif previous_module == 'senses_and_roles':
            for outdb in self.inference_steps[-1].output_dbs:
                for r in outdb.query('action_core(?w, ?a)'):
                    actioncore = r['?a']
                    mod = self.prac.getModuleByName('roles_transformation')
                    plans = mod.getPlanList()
                    if actioncore in plans: return 'plan_generation'
                    else: return 'achieved_by'
        elif previous_module == 'achieved_by':
            return 'roles_transformation'
        elif previous_module == 'roles_transformation':
            if self.inference_steps[-1].module.isLastActionCoreAPlan:
                return 'plan_generation'
            return 'achieved_by'
        elif previous_module == 'plan_generation':
            return None
        
        return None 
    
    def finalgraph(self, filename=None):
        finaldb = Database(self.prac.mln)
        for step in self.inference_steps:
            for db in step.output_dbs:
                for atom, truth in db.evidence.iteritems():
                    if truth == 0: continue
                    _, predname, args = self.prac.mln.logic.parseLiteral(atom)
                    if predname in ActioncoreDescriptionHandler.roles().union(['has_sense', 'action_core', 'achieved_by']):
                        finaldb << atom
#         finaldb.write(sys.stdout, color=True)
        g = Digraph(format='svg', engine='dot')
        g.attr('node', shape='box', style='filled')
        for res in finaldb.query('action_core(?w, ?a) ^ has_sense(?w, ?s)'):
            actioncore = res['?a']
            sense = res['?s']
            predname = 'action_core'
            g.node(actioncore, fillcolor='#bee280')
            g.node(sense)
            g.edge(actioncore, sense, label='is_a')
            roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
            for role in roles:
                for res in db.query('%s(?w, %s) ^ has_sense(?w, ?s)' % (role, actioncore)):
                    sense = res['?s']
                    g.node(sense)
                    g.edge(actioncore, sense, label=role)
        for res in finaldb.query('achieved_by(?a1, ?a2)'):
            a1 = res['?a1']
            a2 = res['?a2']
            g.node(a1, fillcolor='#bee280')
            g.node(a2, fillcolor='#bee280')
            g.edge(a1, a2, label='achieved_by')
            actioncore = a2
            roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
            for role in roles:
                for res in db.query('%s(?w, %s) ^ has_sense(?w, ?s)' % (role, actioncore)):
                    sense = res['?s']
                    g.node(sense)
                    g.edge(actioncore, sense, label=role)
        return render_gv(g, filename)

    
# class PRACInit(PRACReasoner):
#     
#     def __init__(self, actioncore):
# #         super(PRACInit, self).__init__('init')
#         PRACReasoner.__init__(self, 'init')
#         self.prac = PRAC()
#         self.actioncore = self.prac.action_cores[actioncore]
#     
#     @PRACPIPE
#     def run(self):
#         watch = self.pracinference.watch
#         actioncore = self.actioncore
#         self.pracinference.actioncore = actioncore
#         mln = actioncore.learnedMLN.duplicate()
#         self.pracinference.mlns['pracinit'] = mln
#         
#         # generate the feature extractors 
#         features = self.pracinference.features
# 
#         synParser = Syntax(features)
#         features.add(synParser)
#         features.add(WordSenses(features))
#         
#         # feed evidence into the database
#         db = PRACDatabase(mln)
#         self.pracinference.databases['core'] = db
#         feat = []
#         feat.append(features.get('syntax'))
#         feat.append(features.get('wordsenses'))
#         for f in feat:
# #            print "PRINTING F" + f
#             f.run(self.pracinference)
# 
# #        print 
# #        print bash.BOLD + 'Evidence' + bash.END
# #        for e in sorted(db.evidence):
# #            print e
# #        
#         queryPreds = ['has_sense', 'action_role']
#         for pred in mln.predicates:
#             if not pred in queryPreds:
#                 mln.closedWorldPreds.append(pred)
# 
#         # ground the MRF        
#         watch.tag('MRF Construction')
#         mrf = mln.groundMRF(db, simplify=True, method='DefaultGroundingFactory')
#         mln.mrf = mrf
# #        
# #        ##############################################
# #        # start experimental code
# #        ##############################################
# #        
# #         bnb = BranchAndBound(mrf)
# #         bnb.search()
# #         for s in bnb.best_solution:
# #             print s, ':', bnb.best_solution[s]
# #         exit(0)
# #        ##############################################
# #        # start experimental code
# #        ##############################################
#         
#         
#         # convert to WCSP
#         watch.tag('WCSP Construction & Inference')
#         conv = WCSPConverter(mrf)
# 
#         resultDB = PRACDatabase(mln, db=conv.getMostProbableWorldDB())
#         self.pracinference.databases['core'] = resultDB
#         
#         
#         print 
#         print bash.BOLD + 'Syntactic Parser Features:' + bash.END
#         print 
#         for f in synParser.deps:
#             print f
#         
#         print
#         print bash.BOLD + 'Word Sense Disambiguation' + bash.END
#         print bash.BOLD + '=========================' + bash.END
#         for s in resultDB.query('has_sense(?w, ?s) ^ !is_a(?s, NULL)'):
#             print bash.BOLD + s['?w'] + bash.END
#             feature = features.get('wordsenses')
#             senses = feature.words2senses[s['?w']]
#             tick = senses.index(s['?s'])
#             printWordSenses(map(lambda x: feature.senses2concepts[x], senses), tick)
#             print
#         print
#         print
#         print bash.BOLD + 'Action Role Assignment' + bash.END
#         print bash.BOLD + '======================' + bash.END
#         print
#         print bash.BOLD + 'Action Core:' + bash.END + ' ' + actioncore.name
#         print bash.BOLD + 'Roles:' + bash.END
#         printRoles(resultDB, actioncore)
#         print
#         watch.finish()
#         watch.printSteps()
#         print
#         
#     @PRACPIPE    
#     def __call__(self, *args):
#         self.pracinference = PRACInference(self.prac)
#         self.pracinference.sentence = args[0]
#         self.run()
#         
#     def getWordSenses(self, world):
#         """Return the word senses as a dictionary."""
# 
#         result = dict()
#         feature = self.features.get('wordsenses')        
#         for i, word in enumerate(self.database.domains['word']):
#             senseAtoms = feature.senseAtoms.get(word, None)
#             result[word] = dict()
#             if senseAtoms is None:
#                 result[word]['sense'] = 'NULL'
#                 continue
#             for senseAtom in senseAtoms:
#                 tick = world[self.mrf.gndAtoms[senseAtom].idx]
#                 if tick: result[word]['sense'] = re.findall(r"[^(),]+",
#                         senseAtom)[2]
#         return result
#     
#     
# 
# class ActionCores(PRACReasoner):
#     
#     @PRACPIPE
#     def run(self):
#         pass

# class ActionRoles(PRACReasoner):
#     
#     @PRACPIPE
#     def run(self):
#         infer = self.pracinference
#         features = infer.features
#         features.add(MissingRoles(features))
#         
#         watch = infer.watch
#         prac = infer.prac
#         watch.reset()
#         watch.tag('Inferring missing roles')
#         actioncore = infer.actioncore
#         
#         mln = actioncore.learnedMLN.duplicate()
#         db = PRACDatabase(mln)
#         infer.mlns['missingroles'] = mln
#         infer.databases['missingroles'] = db
#         
#         feature = features.get('missingroles')
#         feature.run(infer)
#         
#         print 
#         
#         print bash.BOLD + 'Instruction Completion' + bash.END
#         print bash.BOLD + '======================' + bash.END
# 
#         print
#         print bash.BOLD + 'Missing Roles:' + bash.END
#         for role in feature.missingRoles:
#             print '    ' + bash.BOLD + bash.RED + role + bash.END, '--', actioncore.action_roles[role].definition
#         print
#         if len(feature.missingRoles) == 0:
#             print '    none'
#         queryPreds = ['has_sense', 'has_pos']
#         for pred in mln.predicates:
#             if not pred in queryPreds:
#                 mln.closedWorldPreds.append(pred)
#         
#         mrf = mln.groundMRF(db)
#         converter = WCSPConverter(mrf)
#         
# #         resultDB = PRACDatabase(mln, db=converter.getMostProbableWorldDB())
# #         infer.databases['missingroles'] = resultDB
#         
#         self.pracinference.role_distribution = {}
#         self.pracinference.inferredMissingRoles = {}
#         for role in feature.missingRoles:
#             for concept in actioncore.known_concepts: break
#             self.pracinference.role_distribution[role] = converter.getPseudoDistributionForGndAtom('has_sense(%s,%s)' % (role, concept))
#             sortedList = sorted([(str(l.params[1]),v) for l, v in self.pracinference.role_distribution[role].iteritems()], key=operator.itemgetter(1), reverse=True)
#             printDistribution(sortedList)
#             infer.databases['missingroles'].addGroundAtom('action_role(%s,%s)' % (role, role))
#             infer.databases['missingroles'].addGroundAtom('has_sense(%s,%s)' % (role, sortedList[0][0]))
#             infer.databases['missingroles'].addGroundAtom('is_a(%s,%s)' % (sortedList[0][0], sortedList[0][0]))
#             for c in feature.sense2concepts[sortedList[0][0]]:
#                 infer.databases['missingroles'].addGroundAtom('is_a(%s,%s)' % (sortedList[0][0], c))
#                 
#             self.pracinference.inferredMissingRoles[role] = sortedList[0][0]
#             print 
#             print 'Most likely %s: ' % (role) + bash.OKGREEN + bash.BOLD + sortedList[0][0] + bash.END
#             print
#             print
#             
#                 
#         
# #        for atom, prob in dist.iteritems():
# #            print str(atom) + '\t' + str(prob)
#         print
#         print
#         print bold('Instruction completed.')
#         print
#         print 
#         
# class PRACResult(PRACReasoner):
#     
#     def __init__(self):
# #         super(PRACResult, self).__init__('result')
#         PRACReasoner.__init__(self, 'result')
#     
#     @PRACPIPE
#     def run(self):
#         pass
# 
# actioncores = ActionCores('actioncores')
# actionroles = ActionRoles('actionroles')




    
#     def get_inference_steps_of_module(self, module_name, index=-1):
#         '''
#         Returns the list of InferenceStep instances, which have been
#         performed by the module with the given name at the time point
#         index, where index=-1 stands for the most recent step.
#         '''
#         inf_steps = self.module2infSteps.get(module_name, None)
#         if inf_steps is None:
#             raise Exception('There is no inference steps by the module "%s"' % module_name)
#         return inf_steps[index]
        
#     def to_syntactic_graph(self):
#         '''
#         Returns a json dump of a directed graph representing
#         the syntactic structure of the instruction under consideration.
#         '''
#         # nodes
#         graph = self.graph
#         mln = self.mlns['pracinit'].mrf
#         for word in mln.domains['word']:
#             graph.addNode(Node(id=word))
#         # links
#         links = []
#         for dep in self.features.get('syntax').deps:
#             atom = parseFormula(dep)
#             word1 = graph.getNodeById(atom.params[0])
#             word2 = graph.getNodeById(atom.params[1])
#             if word1 is None or word2 is None: continue
#             graph.addLink(Link(word1, word2, label=atom.predName))
#         # fix the action verb
#         resultDB = self.databases['core']
#         for actionverb in resultDB.query('action_role(?w, ActionVerb)'):
#             graph.getNodeById(actionverb['?w']).fixed = True
#         return graph.toJSON()
#     
#     def to_word_senses(self):
#         sensesFeat = self.features.get('wordsenses')
#         mln = self.mlns['pracinit'].mrf
#         graph = self.graph
#         nodes = set([sense for sense in mln.domains['sense_id'] if sense != 'Nullsense'])
#         for paths in sensesFeat.senses2hypernyms.values():
#             for path in paths:
#                 nodes.update(path)
#         for n in nodes:
#             graph.addNode(Node(id=n))
#         for word, senses in sensesFeat.words2senses.iteritems():
#             for sense in senses:
#                 graph.addLink(Link(graph.getNodeById(word), graph.getNodeById(sense), label='hasSense'))
#         for sense in sensesFeat.senses2hypernyms:
#             for hypernym_path in sensesFeat.senses2hypernyms[sense]:
#                 graph.addLink(Link(graph.getNodeById(sense), graph.getNodeById(hypernym_path[-1]), label='isa'))
#                 previous_concept = None
#                 for concept in hypernym_path:
#                     if previous_concept is None:
#                         previous_concept = concept
#                         continue
#                     graph.addLink(Link(graph.getNodeById(concept), graph.getNodeById(previous_concept), label='isa'))
#                     previous_concept = concept
#         return graph.toJSON()
#     
#     def to_possible_roles(self):
#         sensesFeat = self.features.get('wordsenses')
#         mln = self.mlns['pracinit'].mrf
#         roles = [role for role in mln.domains['role'] if role != 'NULL']
#         links = []
#         nodes = []
#         for word in sensesFeat.words2senses:
#             for role in roles:
#                 nodeid = role+word
#                 self.graph.addNode(Node(id=nodeid, label=role))
#                 self.graph.addLink(Link(self.graph.getNodeById(word), self.graph.getNodeById(nodeid), label='actionRole'))
#         return self.graph.toJSON()
#     
#     def to_senses_and_roles(self):
#         resultDB = self.databases['core']
#         sensesFeat = self.features.get('wordsenses')
#         self.atom2color = {}
#         
#         for s in resultDB.query('has_sense(?w, ?s) ^ !is_a(?s, NULL) ^ action_role(?w, ?r) ^ !(?r=NULL)'):
#             self.color_idx += 1
#             color = self.color_idx
#             self.graph.getNodeById(s['?w']).color = color
#             self.graph.getNodeById(s['?s']).color = color
#             self.graph.getNodeById(s['?r']+s['?w']).color = color
#             self.atom2color[(s['?w'], s['?s'])] = color
#             self.atom2color[(s['?w'], s['?r'])] = color
#         for word, senses in sensesFeat.words2senses.iteritems():
#             for sense in senses:
#                 q = {'label': 'hasSense(%s, %s)' % (word, sense)}
#                 if self.atom2color.get((word, sense), None) is not None:
#                     q['color'] = self.atom2color[(word, sense)]
#                 self.query.append(q)
#             for r in [role for role in self.mlns['pracinit'].mrf.domains['role'] if role != 'NULL']:
#                 q = {'label': 'actionRole(%s, %s)' % (word, r)}
#                 if self.atom2color.get((word, r), None) is not None:
#                     q['color'] = self.atom2color[(word, r)]
#                 self.query.append(q)
#         for dep in self.features.get('syntax').deps:
#             atom = parseFormula(dep)
#             self.evidence.append({'label': '%s(%s, %s)' % (atom.predName, atom.params[0], atom.params[1])})
#         self.evidence.append({'label': 'isa'})
#         self.evidence.append({'label': '...'})
#         return self.graph.toJSON()
#     
# #     def get_sr_symbolic(self):
# #         if len(self.query) == 0:
#             
#     
#     def to_inapplicable_nodes(self):
#         resultDB = self.databases['core']
#         removeNodes = set()
#         keepNodes = []
#         for s in resultDB.query('has_sense(?w, ?s) ^ is_a(?s, ?c) ^ !is_a(?s, NULL)'):
#             keepNodes.extend([s['?c'], s['?w'], s['?s']])
#         for s in resultDB.query('!has_sense(?w, ?s) ^ is_a(?s, ?c) ^ !action_role(?w, ?r) ^ !(?r=NULL)'):
#             removeNodes.update([s['?w'], s['?s'], s['?r']+s['?w'], s['?c']])
#         removeNodes.difference_update(keepNodes)
#         for n in removeNodes:
#             self.graph.removeNodeById(n)
#         return self.graph.toJSON()
#     
#     def to_missing_roles(self):
#         roleFeature = self.features.get('missingroles')
#         resultDB = resultDB = self.databases['core']
#         for role in roleFeature.missingRoles:
#             self.color_idx += 1
#             skolemWord = Node(id=role+role, label='Skolem-'+role)
#             skolemWord.color = self.color_idx
#             self.graph.addNode(skolemWord)
#             for actionverb in resultDB.query('action_role(?w, ActionVerb)'):
#                 av = self.graph.getNodeById(actionverb['?w'])
#                 self.graph.addLink(Link(av, skolemWord, label=''))
#             roleNode = Node(id=role)
#             roleNode.color = self.color_idx
#             self.graph.addNode(roleNode)
#             self.graph.addLink(Link(skolemWord, roleNode, label='actionRole'))
#             
#         return self.graph.toJSON()
#     
#     def to_possible_missing_roles(self):
#         roleFeature = self.features.get('missingroles')
#         g = self.graph
#         mrf = self.mlns['missingroles'].mrf
#         for c in mrf.domains['sense']:
#             node = g.getNodeById(c)
#             if node is None:
#                 node = g.addNode(Node(id=c))
#         for concept in [x for x in self.actioncore.known_concepts if x != 'NULL']:
#             knownSense = Node(id=concept+'_sense', label=concept+'_senseID')
#             g.addNode(knownSense)
#             knownConcept = g.getNodeById(concept)
#             l = Link(knownSense, knownConcept, label='isa')
#             if not g.containsLink(l): g.addLink(l)
#             
#         for role in roleFeature.missingRoles:
#             for concept in [x for x in self.actioncore.known_concepts if x != 'NULL']:
#                 senseNode = g.getNodeById(concept+'_sense')
#                 g.addLink(Link(g.getNodeById(role+role), senseNode, label='hasSense'))
#                 for path in wn.synset(concept).hypernym_paths():
#                     previous = None
#                     for c in path:
#                         if previous is not None:
#                             l = Link(g.getNodeById(c.name), g.getNodeById(previous.name), label='isa')
#                             if not g.containsLink(l):
#                                 g.addLink(l)                             
#                         previous = c
#         return g.toJSON()
#     
#     def to_missing_role_senses(self):
#         keepNodes = []
#         self.evidence = []
#         self.query = []
#         for role in self.inferredMissingRoles:
#             concept = self.inferredMissingRoles[role]
#             keepNodes.append(concept)
#             node = self.graph.getNodeById(concept+'_sense')
#             node.color = self.color_idx
#             self.atom2color[concept] = self.color_idx
#         removeNodes = set()
#         for concept in [x for x in self.actioncore.known_concepts if x != 'NULL' and not x in keepNodes]:
#             removeNodes.add(concept+'_sense')
#         db = self.databases['missingroles']
#         for s in db.query('has_sense(?w, ?s) ^ is_a(?s, ?c) ^ !is_a(?s, NULL)'):
# #             print s['?w'], s['?s'], s['?c']
#             if self.atom2color.get(s['?c'], None) is not None:
#                 q = {'label': 'hasSense(%s, %s)' % (s['?w'], s['?s']), 'color': self.atom2color.get(s['?c'], None)}
#                 if q not in self.query:
#                     self.query.append(q)
#             keepNodes.extend([s['?c']])
#         for s in db.query('!has_sense(?w, ?s) ^ is_a(?s, ?c) ^ !is_a(?s, NULL)'):
#             q = {'label': 'hasSense(%s, %s)' % (s['?w'], s['?s'])}
#             if q not in self.query:
#                 self.query.append(q)
#             removeNodes.update([s['?c']])
#         removeNodes.difference_update(keepNodes)
#         print removeNodes
#         for n in removeNodes:
#             self.graph.removeNodeById(n)
#             
#         for s in db.query('has_sense(?w, ?s) ^ action_role(?w, ?r) ^ !(?r=NULL) ^ !is_a(?s, NULL)'):
#             if self.atom2color.get((s['?w'], s['?r']), None) is None:
#                 continue
#             self.evidence.append({'label': 'hasSense(%s, %s)' % (s['?w'], s['?s']), 'color': self.atom2color[(s['?w'], s['?s'])]})
#             self.evidence.append({'label': 'actionRole(%s, %s)' % (s['?w'], s['?r']), 'color': self.atom2color[(s['?w'], s['?r'])]})
#         self.evidence.append({'label': 'isa'})
#         self.evidence.append({'label': '...'})
#         return self.graph.toJSON()
        

# def printWordSenses(senses, tickIdx):
#     '''
#     Prints the list of word senses and ticks the one specified by the given index.
#     '''
#     for idx, sense in enumerate(senses):
#         if isinstance(sense, basestring):
#             sense = wn.synset(sense)
#         print '    [%s] %s: %s (%s)' % ('X' if tickIdx==idx else ' ', bash.BOLD + sense.name + bash.END, sense.definition, ';'.join(sense.examples))  
# 
# def printRoles(db, actioncore):
#     for var in db.query('action_role(?w, ?r) ^ !(?r=NULL) ^ !(?r=ActionVerb)'):
#         print '    ' + bash.OKGREEN + bash.BOLD + var['?r'] + bash.END + ': ' + var['?w'] + ' (%s)' %  actioncore.action_roles[var['?r']].definition
#     
# 
# def printDistribution(distribution):
#     maxlen = max(map(len, map(lambda x: str(x[0]), distribution)))
#     for l, v in distribution:
#         scale = 100
#         barlen = int(math.floor(v * scale))
#         l = str(l)
#         print l.ljust(maxlen+1) + '[' + bash.OKGREEN + ('|' * barlen).ljust(scale) + bash.END + ']'
