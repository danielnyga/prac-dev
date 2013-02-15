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
    
    def __init__(self, formula, predicate, mrf):
        self.formula = formula
        self.predicate = predicate
        self.mrf = mrf
        
        # determine the independent variable sets
        seen_vars = set()
        variable_sets = []
        literals = []
        seen_assignments = []
        ground_formulas = []
        for literal in formula.iterLiterals():
            if not literal.predName == self.predicate: continue
            vars = set(literal.getVariables(mrf).keys())
            literals.append(literal)
            unseen_vars = vars.difference(seen_vars)
            if len(unseen_vars) > 0:
                variable_sets.append(unseen_vars)
                seen_assignments.append([])
            seen_vars.update(unseen_vars)
        self.literals = literals
        self.variable_sets = variable_sets
        self.seen_assignments = seen_assignments
            
            
    def ground(self, gndAtom):
        new_assignments = []
        for i_lit, lit in enumerate(self.literals):
            litAssignment = self.gndAtom2Assignment(lit, gndAtom)
            if litAssignment is not None:
                assignment = dict([(var, litAssignment[var]) for var in self.variable_sets[i_lit]])
                new_assignments.append(assignment)
#        print 'old', self.seen_assignments
#        print 'new', new_assignments
        if len(new_assignments) == 0:
            return 
#        if len(self.seen_assignments) == 0:
#            self.iterGroundings([1]*len(self.variable_sets), new_assignments)
#        else:
        for c in combinations([[0,1]]*len(self.variable_sets)):
            if sum(c) == 0: continue
            print c    
            self.iterGroundings(c, new_assignments)
        for i_lit, lit in enumerate(self.literals):
#            print 'adding', new_assignments[i_lit], 'to', self.seen_assignments[i_lit]
            self.seen_assignments[i_lit].append(new_assignments[i_lit])
        
            
    def iterGroundings(self, oldOrNew, newAssignments, assignment=None):
        if assignment is None:
            assignment = {}
        if len(oldOrNew) == 0:
#            print assignment
            f = self.formula.ground(mrf, assignment, allowPartialGroundings=True)
            print f
            self.groundFormulas.append()
            return
        if oldOrNew[0]:
            possibleAssignments = [newAssignments[len(oldOrNew) - 1]]
        else:
            possibleAssignments = self.seen_assignments[len(oldOrNew) - 1]
        for assign in possibleAssignments:
            assign = dict(assign)
            assign.update(assignment)
            self.iterGroundings(oldOrNew[1:], newAssignments, assign)
        
            
    def gndAtom2Assignment(self, lit, atom):
        '''
        Returns None if the literal and the atom do not match.
        '''
#        print 'lit:', lit
        if lit.predName != atom.predName: return None
        assignment = {}
        for p1, p2 in zip(lit.params, atom.params):
            if FOL.isVar(p1):
                assignment[p1] = p2
#            elif p1 != p2: return None
        return assignment
    
    
    
    
if __name__ == '__main__':
    
    mln = PRACMLN()
    mln.declarePredicate('foo', ['x', 'y'])
    mln.declarePredicate('bar', ['y','z'])
    
    f = parsePracFormula('foo(?x1,?y1) ^ foo(?x2,Daniel) => foo(?y3,?z)')
    mln.addFormula(f, 1.5)
    
    db = PRACDatabase(mln)
    db.addGroundAtom('foo(X, Fred)')
    db.addGroundAtom('foo(X, Daniel)')
    db.addGroundAtom('bar(Fred, Z)')
    db.addGroundAtom('bar(Bob, Y)')
    
    mrf = mln.groundMRF(db, simplify=False)
    
    print mrf.domains
    fg = FormulaGrounding(f, 'foo', mrf)
    
    previous = None
    for a in sorted(mrf.gndAtoms):
        a = mrf.gndAtoms[a]
        if a != previous:
            pass
        print 'atom:', a
        fg.ground(a)



















    