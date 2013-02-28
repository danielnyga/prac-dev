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

def unify_dicts(x, y):
    z = dict(x)
    z.update(y)
    return z

class FormulaGrounding(object):
    
    def __init__(self, formula, mrf, variable_sets, assignments=None):
        self.formula = formula
        self.mrf = mrf
        if assignments is None:
            self.assignments = [{} for _ in range(len(variable_sets))]
        else:
            self.assignments = assignments
            self.formula = formula.ground(mrf, reduce(unify_dicts, assignments))
        self.variable_sets = variable_sets
    
    def ground(self, assignment):
        idx = self._varset_idx_from_varset(assignment)
        if self.assignments[idx] == {}:
            self.assignments[idx] = assignment
            self.formula = self.formula.ground(mrf, assignment, simplify=True, allowPartialGroundings=True)
            return None
        else:
            new_grounding = FormulaGrounding(self.formula.ground(mrf, 
                    assignment, simplify=True, allowPartialGroundings=True), self.mrf, self.variable_sets)
            for i, a in enumerate(self.assignments):
                new_grounding.assignments[i] = a
            new_grounding.assignments[idx] = assignment
            return new_grounding
        
    def _varset_idx_from_varset(self, varset):
        if set(varset) in self.variable_sets:
            return self.variable_sets.index(set(varset))
        else: return None
        
    def __str__(self):
        return str(self.assignments)
    
    def __repr__(self):
        return str(self)

class GroundingFactory(object):
    
    def __init__(self, formula, predicate, mrf):
        self.formula = formula
        self.predicate = predicate
        self.mrf = mrf
        
        # determine the independent variable sets
        seen_vars = set()
        variable_sets = []
        literals = []
        seen_assignments = []
        formula_groundings = []
        domain_sizes = []
        for literal in formula.iterLiterals():
            if not literal.predName == self.predicate: continue
            vars = set(literal.getVariables(mrf).keys())
            literals.append(literal)
            unseen_vars = vars.difference(seen_vars)
            if len(unseen_vars) > 0:
                variable_sets.append(unseen_vars)
                domain_sizes.append([len(mrf.domains[literal.getVarDomain(v, mrf)]) for v in unseen_vars])
                seen_assignments.append([])
            seen_vars.update(unseen_vars)
        formula_groundings.append(FormulaGrounding(formula, mrf, variable_sets))
        self.literals = literals
        self.variable_sets = variable_sets
        self.seen_assignments = seen_assignments
        self.formula_groundings = formula_groundings
        self.domain_sizes = domain_sizes
            
    def ground(self, gndAtom):
        new_assignments = [{} for _ in range(len(self.variable_sets))]
        for i_lit, lit in enumerate(self.literals):
            litAssignment = self.gndAtom2Assignment(lit, gndAtom)
            if litAssignment is not None:
                print litAssignment
                litAssignment = dict([(var, litAssignment[var]) for var in self.variable_sets[i_lit]])
                if not litAssignment in self.seen_assignments[i_lit]:
                    new_assignments[i_lit] = litAssignment
                else: new_assignments[i_lit] = {}
            else: new_assignments[i_lit] = {}
        print new_assignments
        
        for i, assignment in enumerate(new_assignments):
            if assignment == {}: continue
            new_formula_groundings = []
            seen_local_groundings = []
            for formula_grounding in self.formula_groundings:
                if formula_grounding.assignments[:i] + formula_grounding.assignments[i+1:] in seen_local_groundings: continue  
                self.seen_assignments[i].append(assignment)
                if new_grounding is None: continue
                new_grounding = FormulaGrounding(self.mrf, self.formula.ground(self.mrf, self.formula, assignment)
                seen_local_groundings.append(new_grounding.assignments[:i] + new_grounding.assignments[i+1:])
                new_formula_groundings.append(new_grounding)
            self.formula_groundings.extend(new_formula_groundings)
        print self.formula_groundings
        
    def gndAtom2Assignment(self, lit, atom):
        '''
        Returns None if the literal and the atom do not match.
        '''
        if lit.predName != atom.predName: return None
        assignment = {}
        for p1, p2 in zip(lit.params, atom.params):
            if FOL.isVar(p1):
                assignment[p1] = p2
            elif p1 != p2: return None
        return assignment
    
if __name__ == '__main__':
    
    mln = PRACMLN()
    mln.declarePredicate('foo', ['x', 'y'])
    mln.declarePredicate('bar', ['y','z'])
    
    f = parsePracFormula('foo(?x1,?y1) ^ foo(X,?y2) => foo(?y3,?z)')
    mln.addFormula(f, 1.5)
    
    db = PRACDatabase(mln)
    db.addGroundAtom('foo(X, Fred)')
    db.addGroundAtom('foo(X, Daniel)')
    db.addGroundAtom('bar(Fred, Z)')
    db.addGroundAtom('bar(Bob, Y)')
    
    mrf = mln.groundMRF(db, simplify=False)
    
    print mrf.domains
    fg = GroundingFactory(f, 'foo', mrf)
    
    print fg.variable_sets
    previous = None
    for a in sorted(mrf.gndAtoms):
        a = mrf.gndAtoms[a]
        if a != previous:
            pass
        print 'atom:', a
        fg.ground(a)

#    print 'old', fg.seen_assignments
    for gf in fg.formula_groundings:
        print gf
















    