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
import copy
import math
import sys
import utils

class FormulaGrounding(object):
    '''
    Represents a particular (partial) grounding of a formula with respect to _one_ predicate
    and in terms of disjoint sets of variables occurring in that formula. A grounding of
    the formula is represented as a list of assignments of the independent variable sets.
    It represents a node in the search tree for weighted SAT solving.
    Additional fields:
    - depth:    the depth of this formula grounding (node) in the search tree
                The root node (the formula with no grounded variable has depth 0.
    - children: list of formula groundings that have been generate from this fg.
    ''' 
     
    def __init__(self, formula, mrf, parent=None, assignment=None):
        '''
        Instantiates the formula grounding for a given
        - formula:    the formula grounded in this node
        - mrf:        the MRF associated to this problem
        - parent:     the formula grounding this fg has been created from
        - assignment: dictionary mapping variables to their values
        '''
        self.mrf = mrf
        self.formula = formula
        self.parent = parent
        if parent is None:
            self.depth = 0
        else:
            self.depth = parent.depth + 1
        self.children = []
        self.assignment = assignment
        self.groundings = []
        
    def deepcopy(self, parent, groundingFactory):
        new_node = FormulaGrounding(self.formula, self.mrf, parent, self.assignment)
        depth2nodes = groundingFactory.depth2nodes
        nodelist = depth2nodes.get(new_node.depth, None)
        if nodelist is None:
            nodelist = []
            depth2nodes[new_node.depth] = nodelist
        nodelist.append(new_node)
        for child in self.children:
            new_child = child.deepcopy(new_node, groundingFactory)
            if new_child is not None:
                new_node.children.append(new_child)
        return new_node
        
        
    def ground(self, assignment):
        '''
        Takes an assignment of _one_ particular set of independent variables and
        returns a new FormulaGrounding with that assignment.
        '''
        # calculate the number of ground formulas resulting from
        # the remaining set of free variables
        gf_count = 1
        for var in set(self.formula.getVariables(self.mrf)).difference(assignment.keys()):
            domain = mrf.domains[self.formula.getVarDomain(var, self.mrf)]
            gf_count *= len(domain)
        gf = self.formula.ground(mrf, assignment, allowPartialGroundings=True, simplify=False)
        gf.weight = self.formula.weight
        
        # if the simplified gf reduces to a TrueFalse instance, then
        # we return the costs if it's false, or 0 otherwise.
        if isinstance(gf, FOL.TrueFalse):
            if gf.value: return 0.0 
            else:
                costs = self.formula.weight * gf_count
                return costs
        # if the truth value cannot be determined yet, we return
        # a new formula grounding with the given assignment
        else:
            new_grounding = FormulaGrounding(gf, self.mrf, parent=self, assignment=assignment)
            self.children.append(new_grounding)
            return new_grounding
        
    def __str__(self):
        return str(self.assignment) + '->' + str(self.formula)#str(self.assignment)
    
    def __repr__(self):
        return str(self)

class GroundingFactory(object):
    '''
    Implements a factory for generating the groundings of a formula wrt. one
    specific predicate. The groundings are created successively with one
    particular ground atom of the predicate being presented at a time.
    fields:
    - formula:    the (ungrounded) formula representing the root of the
                  search tree
    - predicate   the predicate name this GroundingFactory deals with
    - mrf:        the respective MRF
    - root:       a FormulaGrounding instance representing the root of the tree
    - costs:      the costs accumulated so far
    '''
    
    def __init__(self, formula, mrf):
        '''
        formula might be a formula or a FormulaGrounding instance.
        '''
        self.mrf = mrf
        if isinstance(formula, FOL.Formula):
            self.formula = formula
            self.root = FormulaGrounding(formula, mrf)
        elif isinstance(formula, FormulaGrounding):
            self.root = formula
            self.formula = formula.formula
        self.domains = {}
        variables = formula.getVariables(mrf)
        for var in variables:
            domain = list(mrf.domains[variables[var]])
            self.domains[var] = domain
        self.values_processed = {}
        for var in variables:
            self.values_processed[var] = []
        self.vars_processed = []
        self.depth2fgs = {0: [self.root]}
    
    def duplicate(self):
        '''
        Returns a deep copy of the GroundingFactory.
        ''' 
        dupl = GroundingFactory(self.formula, self.predicate, self.mrf)
        dupl.literals = list(self.literals)
        dupl.variable_sets = self.variable_sets
        dupl.seen_assignments = copy.deepcopy(self.seen_assignments)
        dupl.depth2nodes = {}
        dupl.root = self.root.deepcopy(None, dupl)
        dupl.varsetIdx2depth = copy.deepcopy(self.varsetIdx2depth)
        dupl.depth2varsetIdx = copy.deepcopy(self.depth2varsetIdx)
        dupl.costs = 0.0
        return dupl
    
    def reset(self):
        self.seen_assignments = {}
        self.depth2nodes = {0: [self.root]}
        self.varsetIdx2depth = {}
        self.depth2varsetIdx = {}    
        self.costs = 0
            
    def ground(self, gndAtom):
        '''
        Expects a ground atom from the predicate of this GroundingFactory
        and creates all groundings that can be derived by it in terms of
        FormulaGroundings. They are stored internally and the accessed via
        the get_all_groundings() method.
        '''
        # get all var assignments of matching literals in the formula 
        var_assignments = {}
        for lit in self.formula.iterLiterals():
            assignment = self.gndAtom2Assignment(lit, gndAtom)
            if assignment is not None:
                utils.unifyDicts(var_assignments, assignment)
        for var in var_assignments.keys():
            if var_assignments[var] in self.values_processed[var]:
                del var_assignments[var]
        # sort the variables according to their depth in the search tree
        for var in var_assignments:
            if not var in self.vars_processed:
                self.vars_processed.append(var)
        vars = var_assignments.keys()#sorted(var_assignments.keys(), key=self.vars_processed.index)
        
        cost = .0
        for var in vars:
            val = var_assignments[var]
            depth = self.vars_processed.index(var) + 1
            queue = list(self.depth2fgs[depth-1])
            while len(queue) > 0:
                fg = queue.pop()
                if fg.depth >= depth and fg.depth < len(self.vars_processed):
                    key_values = [{self.vars_processed[fg.depth]: v} for v in self.values_processed[self.vars_processed[fg.depth]]]
                elif fg.depth < depth:
                    key_values = [{var: val}]
                else: key_values = []
                for key_value in key_values:
                    gnd_result = fg.ground(key_value)
                    if isinstance(gnd_result, FormulaGrounding):
                        if len(gnd_result.formula.getVariables(self.mrf)) == 0:
                            print 'ground formula:', gnd_result.formula
                        if not depth in self.depth2fgs:
                            self.depth2fgs[gnd_result.depth] = []
                        self.depth2fgs[gnd_result.depth].append(gnd_result)
                        queue.append(gnd_result)
                    else: # ...otherwise its true/false; add its costs and discard it.
                        cost += gnd_result
            self.values_processed[var].append(val) 
    
    def printTree(self):
        queue = [self.root]
        while len(queue) > 0:
            n = queue.pop()
            space = ''
            for _ in range(n.depth): space += '--'
            print space + str(n)
            queue.extend(n.children)
        
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
    

class BranchAndBound():
    
    def __init__(self, mrf):
        self.mrf = mrf
        self.upperbound = float('inf')
        self.best_solution = None
        self.vars = []
        self.varIdx2GndAtom = {}
        self.gndAtom2VarIndex = {}
        self.createVariables()
        
    def createVariables(self):
        '''
        Create the variables, one binary for each ground atom.
        Considers also mutually exclusive blocks of ground atoms.
        '''
        handledBlocks = set()
        for gndAtom in self.mrf.gndAtoms.values():
            blockName = self.mrf.gndBlockLookup.get(gndAtom.idx, None)
            if blockName is not None:
                if blockName not in handledBlocks:
                    # create a new variable
                    varIdx = len(self.vars)
                    self.vars.append(blockName)
                    # create the mappings
                    for gndAtomIdx in self.mrf.gndBlocks[blockName]:
                        self.gndAtom2VarIndex[self.mrf.gndAtomsByIdx[gndAtomIdx]] = varIdx
                    self.varIdx2GndAtom[varIdx] = [self.mrf.gndAtomsByIdx[i] for i in self.mrf.gndBlocks[blockName]]
                    handledBlocks.add(blockName)
            else:
                varIdx = len(self.vars)
                self.vars.append(str(gndAtom))
                self.gndAtom2VarIndex[gndAtom] = varIdx
                self.varIdx2GndAtom[varIdx] = [gndAtom]
        self._simplifyVariables()
                
    def _simplifyVariables(self):
        '''
        Removes variables that are already given by the evidence.
        '''
        sf_varIdx2GndAtoms = {}
        sf_gndAtom2VarIdx = {}
        sf_vars = []
        evidence = [i for i, e in enumerate(self.mrf.evidence) if e is not None]
        for varIdx, var in enumerate(self.vars):
            gndAtoms = self.varIdx2GndAtom[varIdx]
            unknownVars = filter(lambda x: x.idx not in evidence, gndAtoms)
            if len(unknownVars) > 0:
                # all gndAtoms are set by the evidence: remove the variable
                sfVarIdx = len(sf_vars)
                sf_vars.append(var)
                for gndAtom in self.varIdx2GndAtom[varIdx]:
                    sf_gndAtom2VarIdx[gndAtom] = sfVarIdx
                sf_varIdx2GndAtoms[sfVarIdx] = self.varIdx2GndAtom[varIdx]
        self.vars = sf_vars
        self.gndAtom2VarIndex = sf_gndAtom2VarIdx
        self.varIdx2GndAtom = sf_varIdx2GndAtoms
        
    def search(self):
        self.factories = [GroundingFactory(f, self.mrf) for f in self.mrf.formulas]
        self._recursive_expand(self.mrf.gndAtoms.values(), [], .0, None)
        
    def _recursive_expand(self, gndAtoms, assignment, lowerbound, lastpredicate):
        # if we have found a solution, update the global upper bound
        space = ''
        for _ in range(len(self.mrf.gndAtoms)-len(gndAtoms)):
            space += '    '
        if len(gndAtoms) == 0: # all gndAtoms have been assigned
            if lowerbound <= self.upperbound:
                print space + 'new solution:', assignment
                print space + 'costs:', lowerbound
                self.upperbound = lowerbound
                self.best_solution = assignment
                return
        if lowerbound >= self.upperbound:
            print space+'backtracking for LB=%f >= UB=%f' % (lowerbound, self.upperbound)
            return
        gndAtom = gndAtoms[0]
        print gndAtom
#        atom = self.varIdx2GndAtom[var][0]
        for factory in self.factories:
            factory.ground(gndAtom)
        self._recursive_expand(gndAtoms[1:], assignment, lowerbound, lastpredicate)
#        
#        for value in values:
#            strVal = value
#            if type(value) != bool:
#                strVal = ' ^ '.join(map(lambda x: str(x) if value==x else '!'+str(x), values))
#            print space+'testing %s = (%s)' % (str(self.vars[var]), strVal)
#            
#            if type(value) != bool:
#                self.mrf._setTemporaryEvidence(value.idx, True)
#                for v in [v for v in values if v != value]:
#                    self.mrf._setTemporaryEvidence(v.idx, False)
#            else:
#                self.mrf._setTemporaryEvidence(self.varIdx2GndAtom[var][0].idx, value)
#            new_gfs = []
#            for gf in groundingFactories:
#                print space+'grounding', gf.formula, 'with', atom
#                new_gf = gf.duplicate()
#                new_gfs.append(new_gf)
#                new_gf.ground(atom)
#                new_gf.printTree()
#            costs = sum(map(lambda x: x.costs, new_gfs))
#            print space + 'costs=%.2f' % (costs+lowerbound)
#            self._recursive_expand(new_gfs, gndAtoms[1:], assignment+[(str(atom), True)], lowerbound+costs, atom.predName)
#            self.mrf._removeTemporaryEvidence()
           
            
if __name__ == '__main__':
    
    mln = PRACMLN()
    mln.declarePredicate('foo', ['x', 'y'], functional=[1])
    mln.declarePredicate('bar', ['y','z'])
    
    f = parsePracFormula('foo(?x1,?y1) ^ foo(?x2,?y1) ^ bar(?y3,Z) ^ bar(?y3, ?z2)')
    mln.addFormula(f, 1.5)
#    mln.addDomainValue('x', 'Z')
    
    db = PRACDatabase(mln)
    db.addGroundAtom('!foo(X, Fred)')
    db.addGroundAtom('!foo(X, Daniel)')
    db.addGroundAtom('!bar(Fred, Z)')
    db.addGroundAtom('bar(Bob, Y)')
    
    
    mrf = mln.groundMRF(db, simplify=False)
#    mrf.evidence = [None for _ in mrf.evidence]
    
#    gfs = []
#    ground_formulas = [FormulaGrounding(f, mrf)]
#    for a in [FOL.GroundAtom('bar', ['Daniel', 'Y']), FOL.GroundAtom('bar', ['Daniel', 'Z'])]:#mrf.gndAtoms.values():
#        print a
#        new_ground_formulas = []
#        for f in ground_formulas:
#            new_ground_formulas.extend(f.spawn(a))
#        for gf in new_ground_formulas:
#            if len(gf.formula.getVariables(mrf)) == 0:
#                gfs.append(gf)
#                new_ground_formulas.remove(gf)
#            print gf.formula
#        ground_formulas = new_ground_formulas
#        print '==========='
#    print 'ground formulas'
#    for gf in sorted(map(lambda x: str(x.formula), gfs)):
#        print gf
#    print '%d GFs' % len(gfs)
#    exit(0)
    bnb = BranchAndBound(mrf)
    print bnb.vars
    bnb.search()
    print 
    print 'Optimal solution found:', bnb.best_solution
    print 'Costs: %.2f' % bnb.upperbound
    exit(0)
    
    formulas = mrf.formulas
    grounding_factories = []
    previous = None
    for gndAtom in sorted(mrf.gndAtoms):
        gndAtom = mrf.gndAtoms[gndAtom]
        print gndAtom
        if gndAtom.predName != previous:
            print gndAtom.predName 
            new_grounding_factories = []
            for f in formulas:
                if type(f) is FormulaGrounding:
                    print f.formula
                    f = f.formula
                gf = GroundingFactory(f, gndAtom.predName, mrf)
                new_grounding_factories.append(gf)
            grounding_factories = new_grounding_factories
            previous = gndAtom.predName
        new_formulas = []
        for gf in grounding_factories:
            gf.ground(gndAtom)
            gf.printTree()
            new_formulas.extend(gf.get_all_groundings())
        formulas = new_formulas
#        
#    fg = GroundingFactory(f, 'foo', mrf)
#    
#    print fg.variable_sets
#    for a in sorted(mrf.gndAtoms):
#        a = mrf.gndAtoms[a]
##        if a != previous:
##            pass
#        print 'atom:', a
#        fg.ground(a)

#    print 'old', fg.seen_assignments
    print 'ground formulas:'
    for gf in formulas:
        print gf.formula

    print len(formulas), 'ground formulas'





#class FormulaGrounding(object):
#    
#    def __init__(self, formula, mrf, forbidden=None, groundings_count=None):
#        self.formula = formula
#        self.mrf = mrf
#        if groundings_count is None:
#            # calculate the number of ground formulas resulting from
#            # the remaining set of free variables
#            groundings_count = 1
#            for var in self.formula.getVariables(self.mrf):
#                domain = mrf.domains[self.formula.getVarDomain(var, self.mrf)]
#                groundings_count *= len(domain)
#        self.groundings_count = groundings_count
#        if forbidden is None:
#            self.forbidden = []
#        else:
#            self.forbidden = forbidden
#    
#    def __str__(self):
#        return str(self.formula) + '\n\tforbidden:' + str(self.forbidden) + '\n\tgroundings: %d' % self.groundings_count
#    
#    def gndAtom2Assignment(self, lit, atom):
#        '''
#        Returns None if the literal and the atom do not match; Or
#        an empty dict if they match but no variable is present.
#        Otherwise it returns a tuple of a list of variable names and a
#        dict containing their assignments.
#        '''
#        if type(lit) is FOL.GroundLit:
#            lit = lit.gndAtom
#        if lit.predName != atom.predName: return None
#        varset = []
#        assignment = {}
#        for p1, p2 in zip(lit.params, atom.params):
#            if FOL.isVar(p1):
#                varset.append(p1)
#                assignment[p1] = p2
#            elif p1 != p2: return None
#        return (varset, assignment)
#    
#    def spawn(self, gndAtom):
#        '''
#        Returns a list of formula groundings that can be derived using
#        this formula grounding and the given ground atom.
#        '''
#        f = self.formula
#        # get all var assignments of matching literals in the formula 
#        lit_assignments = []
#        for lit in f.iterLiterals():
#            match = self.gndAtom2Assignment(lit, gndAtom)
#            if match is not None:
#                (_, lit_assignment) = match
#                lit_assignments.append(lit_assignment)
#        # none of the literal assignment may be a subset or superset of another
#        q = list(lit_assignments)
##        print lit_assignments
#        while len(q) > 0:
#            a1 = q.pop()
#            if len(a1) == 0: lit_assignments.remove(a1)
#            for a2 in list(lit_assignments):
#                if set(a1.keys()).issubset(a2.keys()) and not a1 == a2:
#                    lit_assignments.remove(a1)
#        
#        print 'spawning:', f, 'with', lit_assignments
#        # create a list of all possible groundings that can be derived
#        # given the ground atom.
#        new_groundings = []
##        if reduce(sum, map(len, lit_assignments)) == 0:
##            print 'nothing to spawn'
##            return [self]
#        sizes = []
#        for subset in utils.powerset(lit_assignments):
#            assignment = {}
#            forbidden_assignments = []
##            print '    ', c
#            for assign_it, lit_assignment in zip(c, lit_assignments):
#                if assign_it: 
#                    utils.unifyDicts(assignment, lit_assignment)
#                else:
#                    forbidden_assignment = copy.copy(lit_assignment)
#                    forbidden_assignments.append(forbidden_assignment)
##            print '    ', assignment
#            # check if the assignment is legal
#            if not self.isAssignmentValid(assignment):
#                continue
#            print assignment.keys()
#            new_grounding = FormulaGrounding(f.ground(mrf, assignment, simplify=False, allowPartialGroundings=True), 
#                                             mrf, forbidden_assignments)
#            new_grounding.forbidden.extend(self.forbidden) # inherit forbidden assignments
#            # update all forbidden combinations
#            for forbidden in new_grounding.forbidden:
#                for a in assignment:
#                    if a in forbidden:
#                        del forbidden[a]
#                if len(forbidden) == 0: # remove if empty
#                    new_grounding.forbidden.remove(forbidden)
#            new_groundings.append(new_grounding)
#            print '    ', new_grounding
#        return new_groundings
#    
#    def isAssignmentValid(self, assignment):
#        if len(assignment) == 0:
#            return True
#        for invalid in self.forbidden:
#            valid = False
#            for a in assignment:
#                if invalid.get(a, None) != assignment[a]:
#                    valid = True
#                    break
#            if not valid:
#                return False
#        return True        








    
