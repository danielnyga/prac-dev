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



class FormulaGrounding(object):
    
    def __init__(self, formula, mrf, domains=None):
        self.formula = formula
        self.mrf = mrf
        
        if domains is None: # use the whole domain per default 
            domains = {}
            for var in formula.getVariables(mrf):
                domains[var] = mrf.domains[formula.getVarDomain(var, mrf)]
        self.var_domains = domains
        
    def groundWithGroundAtom(self, gndAtom):
        print self.formula, 'unified with', str(gndAtom)
        self.unifiers = []
        for lit in self.formula.iterLiterals():
            unif = FormulaGrounding.getAtomUnifier(lit, gndAtom)
            if unif is not None:
                self.updateUnifiers(unif)
        print self.unifiers
        for unifier in self.unifiers:
            print self.formula.ground(self.mrf, unifier, simplify=False, allowPartialGroundings=True)
            
    @staticmethod
    def getAtomUnifier(lit, gndAtom):
        if lit.predName != gndAtom.predName:
            return None
        unification = {}
        for p1, p2 in zip(lit.params, gndAtom.params):
            if isVar(p1):
                unification[p1] = p2
            elif p1 != p2:
                return None
        return unification
    
    def updateUnifiers(self, unifier):
        for unif in self.unifiers:
            if not set(unif.keys()).isdisjoint(set(unifier.keys())):
                unif.update(unifier)
                return
        self.unifiers.append(unifier)
        
        
        
if __name__ == '__main__':
    
    mln = PRACMLN()
    mln.declarePredicate('foo', ['x', 'y'])
    mln.declarePredicate('bar', ['y','z'])
    
    f = parsePracFormula('foo(?x1,?y1) ^ foo(?x2,?y1) => bar(?y2,?z)')
    mln.addFormula(f, 1.5)
    
    db = PRACDatabase(mln)
    db.addGroundAtom('foo(X, Fred)')
    db.addGroundAtom('foo(X, Daniel)')
    db.addGroundAtom('bar(Fred, Z)')
    
    mrf = mln.groundMRF(db, simplify=False)
    
#    print 'evidence:'
#    for e, t in zip(mrf.gndAtoms, mrf.evidence):
#        print e, t 
    fg = FormulaGrounding(f, mrf)
    for gndAtom in mrf.gndAtoms.values():
        fg.groundWithGroundAtom(gndAtom)
        
#    gf = fg.formula.ground(mrf, {'?x': 'X', '?z': 'Z', '?y': 'Fred'}, simplify=True, allowPartialGroundings=True)
#    gf.printStructure()
#    print 'grounding:', gf,
#    print 'is', gf.isTrue(mrf.evidence)
    
    



















    