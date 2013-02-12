'''
Created on Apr 20, 2012

@author: nyga
'''
import java
import jpype
import os
from StanfordParser import Parser
import re
from Features import FeatureGenerator, Relation, KeyIdentifier, TupleStore
from WordSenseFeatureGenerator import WordSenseFeatureGenerator
from grammar import parsePracFormula

java.classpath.append(os.path.join('..', '3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))

ignorePOS = ('DT', 'JJ', 'JJR', 'JJS', 'IN')

class SyntacticFeatureGenerator(FeatureGenerator):
    '''
    classdocs
    '''

    def __init__(self, grammar):
        '''
        Constructor
        '''
        self.parser = Parser(grammar)
        
    def getFeatureRelations(self, tupleStore, observables):
#        FeatureGenerator.getFeatureRelations(self, tupleStore, observables)
        
        deps = self.parser.getDependencies(observables, True)
        self.deps = map(str, deps)
        words = set()
        for d in deps:
            f = parsePracFormula(str(d))
            words.update(f.params)
        self.posTags = self.parser.getPOS()
        self.pos = []
        for pos in self.posTags.values():
            if not pos[0] in words:
                continue
            self.pos.append('has_pos(%s,%s)' % (pos[0], pos[1]))
            self.posTags[pos[0]] = pos[1]
    
#        store = TupleStore()
#        relations = [Relation(str(d.reln()), [str(d.gov()), str(d.dep()), 'S']) for d in deps]
#        for r in relations:
#            store.addTuple(r)
#        for p in posTags:
#            if len(store.getRelationsFromKey(p[0])) > 0:
#                if p[1] in ignorePOS:
#                    continue
#                p.extend('S')
#                store.addTuple(Relation('has_pos', p))
#        return store
        

if __name__ == '__main__':
    java.classpath.append(os.path.join('..','..','3rdparty','stanford-parser-2012-02-03','stanford-parser.jar'))
    java.startJvm()

    fg = SyntacticFeatureGenerator('../../3rdparty/stanford-parser-2012-02-03/grammar/englishPCFG.ser.gz')
    sg = WordSenseFeatureGenerator()
    
    store = fg.getFeatureRelations(None, ['Fill oil into the skillet.'])
    senses = sg.getFeatureRelations(store, [mln.domains['sense']])
        
    for disj in store.disjunctions:
        print ' v '.join(map(lambda x: str(x), disj))

    # shutdown the jvm
    java.shutdownJvm()
    
        