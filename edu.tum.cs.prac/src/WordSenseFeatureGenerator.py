'''
Created on Apr 21, 2012

@author: nyga
'''
from Features import FeatureGenerator, Relation
from nltk.corpus import wordnet
import networkx as nx
import matplotlib.pyplot as plt

def convertTagToWordnetPOS(tag): 
    if tag in ['NN', 'NNP', 'NNPS', 'NNS']:
        return 'n'
    if tag in ['VB', 'VBD', 'VBG', 'VBN', 'VBP', 'VBZ']:
        return 'v'
    if tag in ['RB', 'RBR', 'RBS']:
        return 'r'
    if tag in ['JJ', 'JJR', 'JJS']:
        return 'a'
    return None
    

class WordSenseFeatureGenerator(FeatureGenerator):
    '''
    classdocs
    '''

    def __init__(self):
        '''
        Constructor
        '''
        #self.wn = jpype.JPackage('edu.tum.cs.ias.wordnet').WordNet3
        self.graphs = {}
        
    def getFeatureRelations(self, tupleStore, observables):
        FeatureGenerator.getFeatureRelations(self, tupleStore, observables)
        postuples = tupleStore.getTuplesOfRelation('hasPOS')
        postuples = sorted(postuples)
        
        if not observables is None:
            knownSenses = [s.lower() for s in observables[0]]
            knownSenses = map(lambda sense: wordnet.synset(sense[0:-5] + '.' + sense[-4] + '.' + sense[-2:]), knownSenses)
        else:
            knownSenses = []
            #print knownSenses
        
        disj = []
        senseCounter = 1
        for t in postuples:
            tokens = t.args[0].value.split('-') 
            label = tokens[0]
            position = int(tokens[1])
#            print label
            pos = convertTagToWordnetPOS(t.args[1])
            if pos is None:
                rel = Relation('hasSense', [t.args[0].value, 'Nullsense', 'S'])
#                rel.vars.append(1)
                tupleStore.addTuple(rel)
                continue
            
            syns = wordnet.synsets(label, pos)
            
           # print syns
#            if len(syns) == 0:
#                rel = Relation('hasSense', [t.args[0].value, 's', 'S'], True)
#                rel.vars.append(1)
#                continue
            if len(syns) == 0:
                rel = Relation('hasSense', [t.args[0].value, 'Nullsense', 'S'])
                tupleStore.addTuple(rel)
                continue
            
            # generate a taxonomy graph
            self.graphs['%s' % t.args[0]] = generateTaxonomyGraph(syns)
            
            disj = tupleStore.addDisjunction()
            for synset in syns:
                # Create one instance for each possible word sense
#                if not synset.name in knownSenses:
#                    print 'ignoring', synset.name
#                    continue
                senseIDs = sorted([l.name for l in synset.lemmas])
                senseID = '%s%.2d' % (senseIDs[0], senseCounter)
                tuple = Relation('hasSense', [t.args[0].value, senseID, 'S'])
                # Add the hypernymy paths
                # Merge the paths first and then create one isa-relation for each 
                hypernymsPath = synset.hypernym_paths()
                hypernymsPath = set(reduce(list.__add__, hypernymsPath))
                if not observables is None:
                    intersectHyperyms = hypernymsPath.intersection(knownSenses)
                else:
                    intersectHyperyms = hypernymsPath
                if len(intersectHyperyms) == 0:
                    continue
                for p in intersectHyperyms:
                    isa = Relation('isa', [senseID, p.name])
                    tupleStore.addTuple(isa)
                tupleStore.addTupleToDisjunction(tuple, disj)
                senseCounter += 1
            # add possible null sense
            tupleStore.addTupleToDisjunction(Relation('hasSense', [t.args[0].value, 'Nullsense', 'S']), disj)
                
        tupleStore.addTuple(Relation('isa', ['Nullsense', 'NULL']))
                
        return tupleStore


    
            