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
from random import shuffle
import copy
import math
import sys
import utils
from utils import difference_update


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
    - mrf:        the respective MRF
    - root:       a FormulaGrounding instance representing the root of the tree,
                  i.e. an ungrounded formula
    - costs:      the costs accumulated so far
    - depth2fgs   mapping from a depth of the search tree to the corresponding list 
                  of FormulaGroundings
    - vars_processed    list of variable names that have already been processed so far
    - values_processed    mapping from a variable name to the list of values of that vaiable that
                          have already been assigned so far.
    This class maintains a stack of all its fields in order allow undoing groundings
    that have been performed once.
    '''
    
    def __init__(self, formula, mrf):
        '''
        formula might be a formula or a FormulaGrounding instance.
        '''
        self.mrf = mrf
        self.costs = .0
        if isinstance(formula, FOL.Formula):
            self.formula = formula
            self.root = FormulaGrounding(formula, mrf)
        elif isinstance(formula, FormulaGrounding):
            self.root = formula
            self.formula = formula.formula
        variables = formula.getVariables(mrf)
        self.values_processed = {}
        for var in variables:
            self.values_processed[var] = []
        self.vars_processed = []
        self.depth2fgs = {0: [self.root]}
        # initialize the stacks
        self.depth2fgsStack = []
        self.values_processedStack = []
        self.costStack = []
        
    def unground(self):
        '''
        Undoes the most recent grounding action.
        '''
        cost = self.costStack.pop()
        values_processed = self.values_processedStack.pop()
        depth2fgs = self.depth2fgsStack.pop()
        self.costs -= cost
        for d in depth2fgs:
            fgs = self.depth2fgs.get(d, None)
            if fgs is not None:
                difference_update(fgs, depth2fgs[d])
                if len(fgs) == 0: del self.depth2fgs[d]
        for var in values_processed:
            self.vars_processed.remove(var)
            values = self.values_processed.get(var, None)
            if values is not None:
                difference_update(values, values_processed[var])
            
    def ground(self, gndAtom):
        '''
        Expects a ground atom from the predicate of this GroundingFactory
        and creates all groundings that can be derived by it in terms of
        FormulaGroundings.
        '''
        # get all var assignments of matching literals in the formula 
        var_assignments = {}
        for lit in self.formula.iterLiterals():
            assignment = self.gndAtom2Assignment(lit, gndAtom)
            if assignment is not None:
                utils.unifyDicts(var_assignments, assignment)
        # remove the variables that have already been processed
        for var in var_assignments.keys():
            if var_assignments[var] in self.values_processed[var]:
                del var_assignments[var]
        vars_processed = []
        for var in var_assignments:
            if not var in self.vars_processed:
                vars_processed.append(var)
        self.vars_processed.extend(vars_processed)
        vars = var_assignments.keys()
        
        cost = .0
        values_processed = {}
        depth2fgs = {}
        for var in vars:
            values_processed[var] = []
            val = var_assignments[var]
            depth = self.vars_processed.index(var) + 1
            queue = list(self.depth2fgs[depth-1])
            while len(queue) > 0:
                fg = queue.pop()
                if fg.depth >= depth and fg.depth < len(self.vars_processed):
                    vars_values = [{self.vars_processed[fg.depth]: v} 
                                   for v in self.values_processed[self.vars_processed[fg.depth]]]
                elif fg.depth < depth:
                    vars_values = [{var: val}]
                else: vars_values = []
                for var_value in vars_values:
                    gnd_result = fg.ground(var_value)
                    if isinstance(gnd_result, FormulaGrounding):
                        if len(gnd_result.formula.getVariables(self.mrf)) == 0:
                            print 'ground formula:', gnd_result.formula
                        if not gnd_result.depth in depth2fgs:
                            depth2fgs[gnd_result.depth] = []
                        if not gnd_result.depth in self.depth2fgs:
                            self.depth2fgs[gnd_result.depth] = []
                        depth2fgs[gnd_result.depth].append(gnd_result)
                        self.depth2fgs[gnd_result.depth].append(gnd_result)
                        queue.append(gnd_result)
                    else: # ...otherwise it's true/false; add its costs and discard it.
                        cost += gnd_result
            values_processed[var].append(val)
            self.values_processed[var].append(val)
        # update the global tree
        self.costs += cost
        self.costStack.append(cost)
        self.depth2fgsStack.append(depth2fgs)
        self.values_processedStack.append(values_processed)
    
    def printTree(self):
        print self.depth2fgs
        print self.vars_processed
        print self.values_processed
        
#        queue = [self.root]
#        while len(queue) > 0:
#            n = queue.pop()
#            space = ''
#            for _ in range(n.depth): space += '--'
#            print space + str(n)
#            queue.extend(n.children)
        
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
           
            
if __name__ == '__main__':
    
    mln = PRACMLN()
    mln.declarePredicate('foo', ['x', 'y'], functional=[1])
    mln.declarePredicate('bar', ['y','z'])
    
    f = parsePracFormula('foo(?x1,?y1) ^ foo(?x2,?y1) ^ bar(?y3,Y) ^ bar(?y3, ?z2)')
    mln.addFormula(f, 1.5)
#    mln.addDomainValue('x', 'Z')
    
    db = PRACDatabase(mln)
    db.addGroundAtom('!foo(X, Fred)')
    db.addGroundAtom('!foo(X, Daniel)')
    db.addGroundAtom('!bar(Fred, Z)')
    db.addGroundAtom('bar(Bob, Y)')
    
    
    mrf = mln.groundMRF(db, simplify=False)
    
    groundingFactories = [GroundingFactory(f, mrf) for f in mrf.formulas]
    
    atoms = list(mrf.gndAtoms.keys())
    shuffle(atoms)
    for f in mrf.formulas: print f
    print mrf.domains
    for atom in atoms:
        print 'grounding with', atom
        for factory in groundingFactories:
            factory.ground(mrf.gndAtoms[atom])
            factory.printTree()
            factory.unground()
            factory.printTree()
    
#    bnb = BranchAndBound(mrf)
#    print bnb.vars
#    bnb.search()
#    print 
#    print 'Optimal solution found:', bnb.best_solution
#    print 'Costs: %.2f' % bnb.upperbound
#    exit(0)
#    
#    formulas = mrf.formulas
#    grounding_factories = []
#    previous = None
#    for gndAtom in sorted(mrf.gndAtoms):
#        gndAtom = mrf.gndAtoms[gndAtom]
#        print gndAtom
#        if gndAtom.predName != previous:
#            print gndAtom.predName 
#            new_grounding_factories = []
#            for f in formulas:
#                if type(f) is FormulaGrounding:
#                    print f.formula
#                    f = f.formula
#                gf = GroundingFactory(f, gndAtom.predName, mrf)
#                new_grounding_factories.append(gf)
#            grounding_factories = new_grounding_factories
#            previous = gndAtom.predName
#        new_formulas = []
#        for gf in grounding_factories:
#            gf.ground(gndAtom)
#            gf.printTree()
#            new_formulas.extend(gf.get_all_groundings())
#        formulas = new_formulas

#    print 'old', fg.seen_assignments
#    print 'ground formulas:'
#    for gf in formulas:
#        print gf.formula
#    print len(formulas), 'ground formulas'





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








    
