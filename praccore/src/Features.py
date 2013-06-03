'''
Created on Apr 20, 2012

@author: nyga
'''

class FeatureExtractor(object):
    '''
    Abstract class for extracting features out of a natural-language sentence.
    '''
    
    def __init__(self, name):
        self.name = name
        
    def run(self, sentence):
        pass

class Relation(object):
        
    def __init__(self, name=None, args=None, neg=False, vars=None):
        if name is None:
            self.name = ''
        else:
            self.name = name
        if args is None:
            self.args = []
        else:
            self.args = args
        self.neg = neg
        if vars is None:
            self.vars = []
        else:
            self.vars = vars
        
    def __str__(self):
        return '%s%s(%s)' % (['', '!'][self.neg], self.name, ','.join([str(a) for a in self.args]))

class KeyIdentifier(object):
    def __init__(self, value=None):
        self.value = value
    
    def __str__(self):
        return str(self.value)
    
    def __eq__(self, other):
        return str(self) == str(other)
    
class TupleStore(object):
    def __init__(self, tuples=None):
        if tuples is None:
            self.tuples = set()
        else:
            self.tuples = tuples
        self.keySet = set()
        self.keyMap = {}
        self.disjunctions = []
    
    def addDisjunction(self):
        disjunction = []
        self.disjunctions.append(disjunction)
        return disjunction
            
    def addTupleToDisjunction(self, relation, disjunction):
        args = []
        for i,a in enumerate(relation.args):
            if i in relation.vars:
                args.append(KeyIdentifier(a))
                continue
            if not a in self.keyMap.keys():
                self.keyMap[a] = KeyIdentifier(a)
            key = self.keyMap[a]
            args.append(key)
        rel = Relation(relation.name, args)
        rel.vars = relation.vars
        rel.neg = relation.neg
        disjunction.append(rel)
        self.tuples.add(rel)
    
    def addTuple(self, relation):
        disjunction = self.addDisjunction()
        self.addTupleToDisjunction(relation, disjunction)
    
    def getAllKeys(self):
        return self.keyMap.values()
    
    def rename(self, old, new):
        k = self.keyMap[old]
        k.value = new
        self.keyMap.pop(old)
        self.keyMap[new] = k

    def getTuplesOfRelation(self, reln):
        return filter(lambda x: x.name == reln, self.tuples)
    
    def removeKey(self, key):
        self.tuples = filter(lambda t: not (t.args[0] == key or t.args[1] == key), self.tuples)
        self.keyMap.pop(key)
        
    def getRelationsFromKey(self, key):
        return filter(lambda t: t.args[0] == key or t.args[1] == key, self.tuples)

class FeatureGenerator(object):
    '''
    Abstract class for a feature generator.
    '''

    def __init__(self, plKB, params):
        '''
        Constructor
        '''
        self.kb = plKB
    
    def getFeatureRelations(self, tupleStore, observables):
        pass
    
