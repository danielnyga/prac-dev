# TRANSFORMATION OF CLASS TAXONOMIES
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

from nltk.corpus import wordnet as wn

class WordNetConceptManager(object):
    
    def __init__(self, actioncore):
        self.action_core = actioncore
        
    def getHypernyms(self, synset):
        
        paths = synset.hypernynm_paths()
        
        


#class TaxonomyTransformer(object):
#    
#    def __init__(self, mln, isa='is_a'):
#        self.pl = Prolog()
#        self.mln = mln
#        self.mappedMLN = mln.duplicate()
#        self.plFormulas = []
#        self.isa = isa
#        self.isa_new = isa + '_mapped'
##        self.mln.predicates[self.isa_new] = [mln.predicates[self.isa][0], mln.predicates[self.isa][0] + '_mapped']
#        
#    def transform(self, dbs):
#        self.senseDomain = set()
#        self.dbs = dbs
#        for db in dbs:
#            self.senseDomain.update(db.domains[self.mln.predicates[self.isa][1]])
#            for e in db.evidence:
#                f = parsePracFormula(e)
#                assert type(f) is Lit
#        unseen = set()
#        synsets = [wn.synset(s) for s in self.senseDomain if not s == 'NULL']
#        for s in synsets:
#            hyponyms = set(s.hyponyms())
#            newunseens = map(lambda x: x.name, hyponyms.difference(synsets))
#            newunseen = set()
#            if len(newunseens) > 0:
#                newunseen = set([newunseens[0]])
#            unseen.update(newunseen)
#        self.graph = taxonomyGraphFromSynsetIDs(self.senseDomain.union(unseen))
#        dag = self.graph
#        nextNode = dag.getNextNode()
#        self.sensesInGraph = []
#        for n in self.graph.traverse():
#            self.sensesInGraph.append(str(n.id))
#        while nextNode != None:
#            g = nx.DiGraph()
##            print 'processing', nextNode 
#            processNode(dag, nextNode)
#            nextNode = dag.getNextNode()
#        for n in dag.traverse():
#            newchildren = set()
#            for c in n.children:
#                if set([wn.synset(x) for x in c.id.pos()]).issubset(set(synsets)):
#                    newchildren.add(c)
#            n.children = newchildren
##            n.children.add(Node(VarSet(['unknown-%s'%str(n)])))
#            
#        for n in dag.traverse():
#            for c in n.children:
#                g.add_edge(n,c)
#            g.node.get(n,{})['label'] = str(n).replace(',','\n')
#        nx.write_graphml(g, '/home/nyga/code/prac/models/tmp/taxonomy.graphml')
#        
##        # do the graph transformation
##        nextNode = self.graph.getNextNode()
##        while nextNode != None:
##            processNode(self.graph, nextNode)
##            nextNode = self.graph.getNextNode()
#    
#        self._assertNewConcepts(self.graph)
#        self.newDBs = []
#        for db in self.dbs:
#            newDB = self._convertDatabase(db)
#            self.newDBs.append(newDB)
#            
##        for r in self.plFormulas:
##            print r
#        
#    def _assertNewConcepts(self, graph):
#        plFunctors = {}
#        for node in graph.traverse(algo='BFS'):
#            assert len(node.parents) <= 1
#            parent = None
#            for parent in node.parents: pass
#            newPLHead = "%s(C,'%s')" % (self.isa_new, str(node.id))
#            plFunctors[node] = newPLHead
#            newPLString = newPLHead + ' :- '
#            newPLString += ', '.join(["%s(C,'%s')" % (self.isa, l) for l in node.id.pos()])
#            if len(node.id.neg()) > 0:
#                newPLString += ", "
#            newPLString += ', '.join(["not(%s(C,'%s'))" % (self.isa, l) for l in node.id.neg()])
#            if parent is not None: # this should only affect the root node
#                newPLString += ', ' + plFunctors[parent]
##            print newPLString
#            self.plFormulas.append(newPLString)
#            self.pl.assertz(newPLString)
#        for s in self.senseDomain:
#            if not s in self.sensesInGraph:
#                newPLString = "%s(C,'%s') :- %s(C, '%s')" % (self.isa_new, s, self.isa, s)
#                self.plFormulas.append(newPLString)
#                self.pl.assertz(newPLString)
#
#    def _convertDatabase(self, db):
#        newDB = PRACDatabase(self.mln)
#        senseIDs = []
#        for e in db.evidence:
#            f = parsePracFormula(e)
#            assert type(f) is Lit
#            predName = f.predName
#            params = f.params
#            if f.predName == self.isa:
#                if not params[0] in senseIDs:
#                    senseIDs.append(params[0])
#                astr = "%s(%s)" % (self.isa, ','.join(["'%s'" % p for p in f.params]))
#                self.pl.asserta(astr)
#            else:    
#                newDB.addGroundAtom(e)
#        for sense in senseIDs:
#            for s in self.pl.query("%s('%s', C)." % (self.isa_new, sense)):
#                newDB.addGroundAtom("%s(%s, %s)" % (self.isa, sense, s['C']))
#        for sense in senseIDs:
#            for s in self.pl.query("retract(%s('%s',C))." % (self.isa, sense)): pass
#            for s in self.pl.query("retract(%s('%s',C))." % (self.isa_new, sense)): pass
#        return newDB
#    
#    def _mapConcept(self, isaAtoms):
#        newIsa = []
#        senseIDs = []
#        for isa in isaAtoms:
#            f = parsePracFormula(isa)
#            assert type(f) is Lit
#            predName = f.predName
#            params = f.params
#            if f.predName == self.isa:
#                if not params[0] in senseIDs:
#                    senseIDs.append(params[0])
#                for i, p in enumerate(f.params):
#                    f.params[i] = p.replace("'", "''")
#                astr = "%s(%s)" % (self.isa, ','.join(["'%s'" % p for p in f.params]))
##                print 'asserting', astr
#                self.pl.asserta(astr)
#        for sense in senseIDs:
#            for s in self.pl.query("%s('%s', C)." % (self.isa_new, sense)):
##                newIsa.append("%s(%s, %s)" % (self.isa_new, sense, s['C']))
#                newIsa.append(s['C'])
#        for sense in senseIDs:
#            for s in self.pl.query("retract(%s('%s',C))." % (self.isa, sense)): pass
#            for s in self.pl.query("retract(%s('%s',C))." % (self.isa_new, sense)): pass
#        return newIsa
#    
##    def generateFormulas(self):
##        conditions = {}
##        for node in self.graph.traverse(algo='BFS'):
##            assert len(node.parents) <= 1
##            if len(node.children) == 0:
##                continue
##            predicates = node.children
##            prefix = 'hasSense(w,s,i)'
##            for c in product([' ','!'], repeat=len(predicates)):
##                formulaString = prefix
##                formulaString += ' ^ ' + ' ^ '.join(['%sisaNew(s,%s)' % (c[i], toMLNCompatibleFormat(str(n.id))) for i, n in enumerate(predicates)])
##                for tf in (' ', '!'):
##                    condition = '%sisaNew(s,%s)' % (tf, toMLNCompatibleFormat(str(node.id)))
###                    print 'conditional:', prefix + ' ^ ' + condition
##                    conditionalGroundings = countTrueGroundings(self.newDBs, prefix + ' ^ ' + condition)[0]
###                    print conditionalGroundings
###                    formulaString += ' ^ ' + condition
##                    formula = formulaString + ' ^ ' + condition
###                    print 'formula:', formula
##                    groundings = countTrueGroundings(self.newDBs, formula)[0]
##                    if groundings == conditionalGroundings:
##                        continue
##                    print '%f' % logx(float(groundings) / conditionalGroundings), formula
##                    f = parsePracFormula(formula)
##                    f.weight = logx(float(groundings) / conditionalGroundings)
##                    f.isHard = False
##                    addFormulaToMLN(self.mln, f)
##        mln.materializeFormulaTemplates(self.newDBs)
#    
#    def writePLMapping(self, filename):
#        f = open(filename, 'w+')
#        for l in self.plFormulas:
#            f.write(l + '.\n')
#        f.close()
#        
#    def loadPLMapping(self, filename):
#        self.pl.consult(filename)
#        
#    def assertPLFormulas(self, formulas):
#        for f in formulas:
#            self.pl.assertz(f)
#    
#    def getNewSenses(self, w, pos):
#        conv = []
#        for i,s in enumerate(wn.synsets(w, pos)):
#            paths = s.hypernym_paths()
#            isa = set()
#            for p in paths:
#                isa.update(p)
#            isa = ['%s(W%d,%s)' % (self.isa, i+1, x.name) for x in isa]
##            print isa
#            conv.append(self._mapConcept(isa))
#        return conv