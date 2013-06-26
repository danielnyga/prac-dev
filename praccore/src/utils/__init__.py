import os
import math 
import time

class StopWatch(object):
    
    def __init__(self):
        self.start = 0
        self.tags = []
        
    def tag(self, label, verbose=True):
        if verbose:
            print label + '...'
        now = time.time()
        self.start = now
        if len(self.tags) > 0:
            self.tags[-1][0] = now - self.tags[-1][0]
        self.tags.append([now, label])
    
    def finish(self):
        now = time.time()
        if len(self.tags) > 0:
            self.tags[-1][0] = now - self.tags[-1][0]
    
    def reset(self):
        self.tags = []
        self.start = time.time()
        
    def printSteps(self):
        for t in self.tags:
            print '%s%s%s took %f sec.' % (bash.BOLD, t[1], bash.END, t[0])

def powerset(seq):
    '''
    Returns all the subsets of this set.
    '''
    if len(seq) <= 1:
        yield seq
        yield [] 
    else: 
        for item in powerset(seq[1:]):
            yield [seq[0]]+item 
            yield item

def unifyDicts(d1, d2):
    '''
    Adds all key-value pairs from d2 to d1.
    '''
    for key in d2:
        d1[key] = d2[key]

def dict_get(d, entry):
    e = d.get(entry, None)
    if e is None:
        e = {}
        d[entry] = e
    return e

def list_get(d, entry):
    e = d.get(entry, None)
    if e is None:
        e = []
        d[entry] = e
    return e

def difference_update(l1, l2):
    '''
    Removes all elements in l1 from l2.
    '''
    for e in l2:
        l1.remove(e)
        
class bash:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    ORANGE = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'
    BOLD = '\033[1m'
    
def red(s):
    return bash.RED + s + bash.END

def bold(s):
    return bash.BOLD + s + bash.END

def green(s):
    return bash.OKGREEN + s + bash.END

def orange(s):
    return bash.ORANGE + s + bash.END



#def getMostSpecificSenses(synsets):
#    concepts = set(synsets)
#    for synset in synsets:
#        hypernympaths = synset.hypernym_paths()
#        for path in hypernympaths:
#            concepts.difference_update(set(path[:-1]))
#    return concepts
#    
#    
#def getIsaAtomsForSynset(synset):
#        hypernyms = set()
#        atoms = []
#        for path in synset.hypernym_paths():
#            hypernyms.update(path)
#        for hyp in hypernyms:
#            
#            atom = "is_a(tmp, %s)" % hyp.name
#            atoms.append(atom)
#        return atoms
#    
#def taxonomyGraphFromSynsetIDs(synsetIDs):
#    nodes = {}
#    entity = wn.synset('entity.n.01')
#    root = Node(VarSet([entity.name]), entity)
#    nodes[entity.name] = root
#    dag = DAG(root)
#    synsets = [wn.synset(s) for s in synsetIDs if not s == 'NULL']
#    
#    for synset in synsets:
#        paths = synset.hypernym_paths()
#        for path in paths:
#            prevNode = None
#            for s in path:
#                node = nodes.get(s.name, None)
#                if node is None:
#                    node = Node(VarSet([s.name]), s)
#                    nodes[s.name] = node
#                if prevNode is not None:
#                    prevNode.addChild(node)
#                prevNode = node
#    return dag
#
#def generateTaxonomyGraph(synsets):
#        '''
#            takes a synset and creates an networkx graph of the taxonomy of synset paths.
#        '''
#        g = nx.DiGraph()
#        for i, synset in enumerate(synsets):
#            paths = synset.hypernym_paths()
#            for path in paths:
#                path.reverse()
#                nodeIDs = map(lambda x: toMLNCompatibleFormat(x.name), path)
#                g.add_path(nodeIDs)
#                #print nodeIDs
#                for node, synset in zip(nodeIDs, path):
#                    g.node.get(node,{})['label'] = node
#                    g.node.get(node,{})['tooltip'] = synset.definition
##        nx.write_graphml(g, 'wordnet.graphml')
#        return g
    
#def colorTaxonomyGraph(mln, g):
#    '''
#        Sets the color of concept nodes in a taxonomy graph to orange,
#        if the concept is contained in the MLN model, or to red, otherwise.
#    '''
#    senseDom = mln.domains['sense']
#    for node in g.nodes():
#        g.node.get(node,{})['color'] = 'orange' if node in senseDom else 'red'
#            
#            
#def getRoleOfWord(results, word):
#    '''
#        Takes an inference result and returns the role assigned to the given word.
#    '''
#    for r in filter(lambda x: x.value == 1., results):
#        if r.ga.predicate == 'hasRole' and r.ga.args[0] == word:
#            return r.ga.args[1]

def logx(x):
    if x < 1E-30:
        return -200
    else:
        return math.log(x)
        
def combinations(domains):
    return _combinations(domains, [])

def _combinations(domains, comb):
    if len(domains) == 0:
        yield comb
        return
    for v in domains[0]:
        for ret in _combinations(domains[1:], comb + [v]):
            yield ret

#def getUnifiedSiblings(synsets, knownSynsets):
#    '''
#        Returns VarSet of the mentioned set of synsets
#        Both parameters must be iterable. 
#    '''
#    # get union of all known hypernyms
#    hypernyms = set()
#    for synset in synsets: 
#        hypernyms.update(synset.hypernyms())
#    hypernyms = hypernyms.intersection(knownSynsets)
#    # get the known subset of the union of their children
#    siblings = set()
#    for hn in hypernyms:
#        siblings.update(hn.hyponyms())
#    siblings = siblings.intersection(knownSynsets)
#    # remove the current synsets themselves to get actual siblings
#    siblings = siblings.difference(synsets)
#    # remove synsets that are actually ancestors of hypernyms
#    transitiveHypernyms = set()
#    for hyp in synsets:
#        for path in hyp.hypernym_paths():
#            transitiveHypernyms.update(path)
#    siblings = siblings.difference(transitiveHypernyms)
#    return siblings

#def getUnifiedHypernymsAndDisjointHypernyms(synsets, knownSynsets):
#    siblings = getUnifiedSiblings(synsets, knownSynsets)
#    nonhypernyms = set()
#    for sibling in siblings: 
#        nonhypernyms.update(sibling.hypernyms())
#    hypernyms = set()
#    for synset in synsets:
#        hypernyms.update(synset.hypernyms())
#    hypernyms = hypernyms.intersection(knownSynsets)
#    nonhypernyms = nonhypernyms.intersection(knownSynsets)
#    nonhypernyms =  nonhypernyms.difference(hypernyms).difference(synsets)
#    return VarSet(hypernyms, nonhypernyms)




#class CondTaxProb(object):
#    '''
#        Represents the probability of a particular (or joint)
#        sense(s) given its (their) direct hypernyms and their disjoint hypernyms
#    '''
#    
#    def __init__(self, querySenses, evidenceSenses=None):
#        assert(type(querySenses) is VarSet and type(evidenceSenses) is VarSet)
#        self.querySenses = querySenses
#        if evidenceSenses is None:
#            self.evidenceSenses = VarSet()
#        else:
#            self.evidenceSenses = evidenceSenses
#        self.prob = None
#    
#    def combine(self, other):
#        if type(other) == list:
#            for o in other: self.combine(o)
#        self.evidenceSenses.unify(other.evidenceSenses)
#        self.querySenses.unify(other.querySenses)
#    
#    def __hash__(self):
#        return (self.querySenses, self.evidenceSenses).__hash__()
#    
#    def __eq__(self, other):
#        return self.evidenceSenses == other.evidenceSenses and self.querySenses == other.querySenses
#    
#    def __str__(self):
#        return 'P(%s|%s)' % (self.querySenses, self.evidenceSenses)
#
#def getKnownHypernyms(synsets, knownSynsets):
#    if type(synsets) is Synset:
#        synset = [synsets]
#    hypernyms = set()
#    for synset in synsets:
#        hypernyms.update(synset.hypernyms())
#    return hypernyms.intersection(knownSynsets)
#
#def getKnownNyponyms(synsets, knownSynsets):
#    if type(synsets) is Synset:
#        synset = [synsets]
#    hyponyms = set()
#    for synset in synsets:
#        hyponyms.update(synset.hyponyms())
#    return hyponyms.intersection(knownSynsets)

#def factorizeConceptProb(synset, knownSynsets):
#    paths = synset.hypernym_paths()
#    probs = set()
#    for p in paths:
#        recent = None
#        for c in p:
#            if recent is not None:
#                prob = CondTaxProb(VarSet([c]), VarSet([recent]))
#                probs.add(prob)
#            recent = c
#    # merge the conditional probabilities
#    candidates = set(probs)
#    while len(candidates) > 0:
#        p1 = candidates.pop()
#        new_probs = set()
#        while len(probs) > 0:
#            p2 = probs.pop()
#            if p1.querySenses.isDisjoint(p2.querySenses) and p1.evidenceSenses.isDisjoint(p2.evidenceSenses):
#                new_probs.add(p2)
#            else:
#                p1.combine(p2)
#        new_probs.add(p1)
#        probs = new_probs
#    for p in probs: # add the disjoint siblings
#        p.querySenses = VarSet(p.querySenses.pos(), getUnifiedSiblings(p.querySenses.pos(), knownSynsets))
#        p.EvidenceSenses = VarSet(p.evidenceSenses.pos(), getUnifiedSiblings(p.evidenceSenses.pos(), knownSynsets))
#    return probs
#    for p in probs:
#        print p
                
#def getCondSenseProbs(synsetIDs, knownSynsetIDs):
#    # transform the mln synsets into nltk-wordnet compatible ones
#    knownSynsets = set([wn.synset(mlnSynsetToWNSynset(x)) for x in knownSynsetIDs if not x == 'NULL']) # set of synsets
#    synsets = set([wn.synset(mlnSynsetToWNSynset(s)) for s in synsetIDs if not s == 'NULL']) # 
#    condProbs = set()
#    for synset in synsets:
#        condProbs.update(factorizeConceptProb(synset, knownSynsets))
#    print condProbs
#    return condProbs
    # and create the inital set of VarSets
#    queryJointVariables = set()
#    for s in synsetIDs:
#        if s == 'NULL': continue
#        synsets = set([wn.synset(mlnSynsetToWNSynset(s))]) # set with one element
#        #sibl = getUnifiedHypernymsAndDisjointHypernyms(synsets, knownSynsets)#getUnifiedSiblings(synsets, knownSynsets)
#        vSet = VarSet(synsets)
#        queryJointVariables.add(vSet)
#    
#    queryJointVariablesDone = set() # set of VarSets
#    condProbs = set() # set of CondTaxProbs
#    while len(queryJointVariables) > 0:
#        queryVars = queryJointVariables.pop()
#        evidenceVars = getUnifiedHypernymsAndDisjointHypernyms(queryVars.pos().union(queryVars.neg()), knownSynsets)
#        queryVars.remove(evidenceVars)
#        if queryVars.isEmpty() or queryVars in queryJointVariablesDone:
#            continue
#        condProb = CondTaxProb(queryVars, evidenceVars)
#        condProbs.add(condProb)
#        queryJointVariables.add(evidenceVars)
#        queryJointVariablesDone.add(queryVars)
#    return condProbs


#def getFormula(vars):
#    # generate a formula
#    if len(vars.pos()) == 0 and len(vars.neg()) == 0:
#        return ''
#    result = 'hasSense(w,s1,s)'# ^ isa(s1, %s)' % toMLNCompatibleFormat(synset.name)
#    if not len(vars.pos()) == 0:
#        result += ' ^ ' + ' ^ '.join(map(lambda x: 'isa(s1, %s)' % toMLNCompatibleFormat(x.name), vars.pos()))
#    if not len(vars.neg()) == 0:
#        result += ' ^ ' + ' ^ '.join(map(lambda x: '!isa(s1, %s)' % toMLNCompatibleFormat(x.name), vars.neg()))
##    print 'P(%s | %s, %s)' % (toMLNCompatibleFormat(synset.name), ', '.join(map(toMLNCompatibleFormat, map(lambda x: x.name, hypernyms))),\
##                               ', '.join(map(lambda x: '!' + x, map(toMLNCompatibleFormat, map(lambda x: x.name, nonhypernyms)))))
#    return result
#
#if __name__ == '__main__':
#    vs1 = VarSet(['bla', 'bli'], ['!bla', '!bli'])
#    vs2 = VarSet(['blub', 'blib'], ['!blub', '!blib'])
#    vs1.unify(vs2)
#    print vs1
#    
#                
#    
