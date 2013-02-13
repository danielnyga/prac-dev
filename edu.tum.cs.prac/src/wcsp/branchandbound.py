# Markov Logic Networks -- Branch-and-Bound Search for MPE inference
#
# (C) 2011-2013 by Daniel Nyga (nyga@cs.tum.edu)
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
from pracmln.PRACMLN import PRACMLN
from pracmln.PRACDatabase import PRACDatabase
from grammar import *
import grammar
from utils import combinations



class FormulaGrounding(object):
    
    def __init__(self, formula, mrf, domains=None):
        self.formula = formula
        self.mrf = mrf
        
        if domains is None: # use the whole domain per default 
            domains = {}
            for var in formula.getVariables(mrf.mln):
                domains[var] = mrf.domains[formula.getVarDomain(var, mrf.mln)]
        self.var_domains = domains
        
    def groundWithGroundAtom(self, gndAtom, lastPredicate):
        self.unifiers = []
        for lit in self.formula.iterLiterals():
            unif = FormulaGrounding.getAtomUnifier(lit, gndAtom)
            if unif is not None:
                self.updateUnifiers(unif)
        print self.formula,
        print self.unifiers,
        if len(self.formula.getVariables(self.mrf.mln)) == 0 or (len(self.unifiers) == 0 and not lastPredicate):
            yield FormulaGrounding(self.formula, self.mrf)
        elif len(self.unifiers) == 1:
            self.unifiers = [self.unifiers]
        else:
            self.unifiers = [(u,{}) for u in self.unifiers]
        for unifier in combinations(self.unifiers):
            unif = {}
            for u in unifier: unif.update(u)
            if len(unif) == 0: continue
            f = self.formula.ground(self.mrf, unif, simplify=False, allowPartialGroundings=True)
            yield FormulaGrounding(f, self.mrf)
            
    @staticmethod
    def getAtomUnifier(lit, gndAtom):
        predName = lit.predName if type(lit) is Lit else lit.gndAtom.predName
        params = lit.params if type(lit) is Lit else lit.gndAtom.params
        if predName != gndAtom.predName:
            return None
        unification = {}
        for p1, p2 in zip(params, gndAtom.params):
            if isVar(p1):
                unification[p1] = p2
            elif p1 != p2:
                return None
        return unification
    
    def updateUnifiers(self, unifier):
        unifiersToMerge = []
        self.unifiers.append(unifier)
        for i, u in enumerate(self.unifiers):
            if set(u).issubset(set(unifier)) or set(unifier).issubset(set(u)):
                unifiersToMerge.append(i)
#        print self.unifiers
        if len(unifiersToMerge) == 0:
            return
        mergedUnifier = {}
        for i in unifiersToMerge:
            mergedUnifier.update(self.unifiers[i])
        self.unifiers = [u for i, u in enumerate(self.unifiers) if i not in unifiersToMerge]
        self.unifiers.append(mergedUnifier)

    def __str__(self):
        return str(self.formula)

class LazyBnB(object):
    
    def __init__(self, mrf):
        self.mrf = mrf
        
    def solve(self):
        mrf = self.mrf
        gndAtoms = mrf.gndAtoms
        newFormulaGroundings = [FormulaGrounding(f, mrf) for f in mrf.formulas]
        formulaGroundings = None
        recentPredicate = None
        reruns = {}
        predicates = list(mrf.mln.predicates)
        for gndAtom in sorted(gndAtoms, reverse=True) + [None]:
            gndAtom = gndAtoms.get(gndAtom, None)
            if gndAtom is None or recentPredicate != gndAtom.predName:
                if len(reruns) > 0:
                    print 'rerunning'
                    newNewFormulaGroundings = []
                    atomList = list(reruns)
                    print atomList[0]
                    for reruni, rerun in enumerate(atomList):
                        for ai, atom in enumerate([a for a in gndAtoms.values() if a.predName == recentPredicate]):
                            print 'enter loop'
                            print map(str,reruns[rerun])
#                            if ai <= reruni: continue
                            print atom
                            for i, fg in enumerate(reruns[rerun]):
                                print fg
                                for gf in fg.groundWithGroundAtom(atom, True):
                                    print '-> adding'
                                    newNewFormulaGroundings.append(gf)
#                        newNewFormulaGroundings.extend(reruns[rerun])
                    newFormulaGroundings.extend(newNewFormulaGroundings)
                    reruns = {}
                    print 'stop rerunning'
                formulaGroundings = newFormulaGroundings
                if gndAtom is None:
                    break
                if gndAtom.predName in predicates:
                    predicates.remove(gndAtom.predName)
                recentPredicate = gndAtom.predName
                newFormulaGroundings = []
            for fg in formulaGroundings:
                newOnes = []
                for gf in fg.groundWithGroundAtom(gndAtom, len(predicates) == 0):
                    print '-> adding'
                    newOnes.append(gf)
                if len(newOnes) > 1: 
                    reruns[gndAtom] = newOnes[1:]
                    newFormulaGroundings.append(newOnes[0])
                else:
                    newFormulaGroundings.extend(newOnes)
        formulaGroundings = newFormulaGroundings
        
        print 'FINAL'
        for gf in formulaGroundings:
            print gf
                
if __name__ == '__main__':
    
    mln = PRACMLN()
    mln.declarePredicate('foo', ['x', 'y'])
    mln.declarePredicate('bar', ['y','z'])
    
    f = parsePracFormula('foo(?x1,Daniel) ^ foo(X,?y2) => bar(?y2,?z)')
    mln.addFormula(f, 1.5)
    
    db = PRACDatabase(mln)
    db.addGroundAtom('foo(X, Fred)')
    db.addGroundAtom('foo(X, Daniel)')
    db.addGroundAtom('bar(Fred, Z)')
    db.addGroundAtom('bar(Bob, Y)')
    
    mrf = mln.groundMRF(db, simplify=False)
    print sorted(mrf.gndAtoms, reverse=True)
    print mrf.domains
    bnb = LazyBnB(mrf)
    bnb.solve()
    
#    print 'evidence:'
#    for e, t in zip(mrf.gndAtoms, mrf.evidence):
#        print e, t 
#    fg = FormulaGrounding(f, mrf)
#    for gndAtom in mrf.gndAtoms.values():
#        fg.groundWithGroundAtom(gndAtom)
        
#    gf = fg.formula.ground(mrf, {'?x': 'X', '?z': 'Z', '?y': 'Fred'}, simplify=True, allowPartialGroundings=True)
#    gf.printStructure()
#    print 'grounding:', gf,
#    print 'is', gf.isTrue(mrf.evidence)
    
    



















    