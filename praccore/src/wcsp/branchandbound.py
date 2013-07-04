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
from logic import FOL
from logic import grammar
from random import shuffle
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
        self.costs = 0.
        if parent is None:
            self.depth = 0
        else:
            self.depth = parent.depth + 1
        self.children = []
        self.assignment = assignment
        self.domains = {}
        if parent is None:
            for var in self.formula.getVariables(self.mrf):
                self.domains[var] = list(self.mrf.domains[self.formula.getVarDomain(var, self.mrf)])
        else:
            for (v, d) in parent.domains.iteritems():
                self.domains[v] = list(d)
    
    def countGroundings(self):
        '''
        Computes the number of ground formulas subsumed by this FormulaGrounding
        based on the domain sizes of the free (unbound) variables.
        '''
        gf_count = 1
        for var in self.formula.getVariables(self.mrf):
            domain = self.domains[var]
            gf_count *= len(domain)
        return gf_count
    
    def ground(self, assignment=None):
        '''
        Takes an assignment of _one_ particular variable and
        returns a new FormulaGrounding with that assignment. If
        the assignment renders the formula false or true, then
        the costs are returned.
        '''
        # calculate the number of ground formulas resulting from
        # the remaining set of free variables
        if assignment is None:
            assignment = {}
        gf_count = 1
        for var in set(self.formula.getVariables(self.mrf)).difference(assignment.keys()):
            domain = self.domains[var]#self.mrf.domains[self.formula.getVarDomain(var, self.mrf)]
            gf_count *= len(domain)
        gf = self.formula.ground(self.mrf, assignment, allowPartialGroundings=True, simplify=True)
#         self.mrf.printEvidence()
#         print 'grounded', gf
        gf.weight = self.formula.weight
        # if the simplified gf reduces to a TrueFalse instance, then
        # we return the costs if it's false, or 0 otherwise.
        if isinstance(gf, FOL.TrueFalse):
            if gf.value: costs = 0.0
            else:
                costs = self.formula.weight * gf_count
            self.costs += costs
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
    Implements a factory for generating the groundings of one formula. 
    The groundings are created successively with one
    particular ground atom being presented at a time.
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
        self.values_processed = {}
        self.variable_stack = [None]
        self.var2fgs = {None: [self.root]}
        self.gndAtom2fgs = {}
        # initialize the stacks
        self.var2fgsStack = []
        self.values_processedStack = []
        self.costStack = []
        
    def unground(self):
        '''
        Undoes the most recent grounding action.
        '''
        costs2remove = self.costStack.pop()
        values_processed2remove = self.values_processedStack.pop()
        var2fgs2remove = self.var2fgsStack.pop()
        self.costs -= costs2remove
        for d in var2fgs2remove:
            fgs = self.var2fgs.get(d, None)
            if fgs is not None:
                difference_update(fgs, var2fgs2remove[d])
                if len(fgs) == 0: self.var2fgs.pop(d)
        for var in values_processed2remove:
            values = self.values_processed.get(var, None)
            if values is not None:
#                 print var, ': removing', values_processed2remove[var], 'from', values
                difference_update(values, values_processed2remove[var])
                if len(values) == 0:
#                     print self.variable_stack
#                     print var
                    if var in self.variable_stack:
                        self.variable_stack.remove(var)
                    self.values_processed.pop(var)
    
    def getVariableDepth(self, varname):
        if not varname in self.values_processed:
            self.values_processed[varname] = []
        if not varname in self.variable_stack:
            self.values_processed[varname] = []
            depth = len(self.variable_stack)
        else:
            depth = self.variable_stack.index(varname)
        return depth
    
    def ground(self, gndAtom):
        '''
        Expects a ground atom and creates all groundings 
        that can be derived by it in terms of FormulaGroundings.
        '''
        # get all variable assignments of matching literals in the formula 
        var_assignments = {}
        for lit in self.formula.iterLiterals():
            assignment = self.gndAtom2Assignment(lit, gndAtom)
            if assignment is not None:
                utils.unifyDicts(var_assignments, assignment)
        cost = .0
        var2fgs = {}
        values_processed = {}
        # first evaluate formula groundings that contain 
        # this gnd atom as an artifact
        min_depth = None
        min_depth_fgs = []
        for fg in self.gndAtom2fgs.get(gndAtom, []):
            # yes, this is a dirty hack; but it needs substantial amount of 
            # refactoring of the FOL.isTrue method to resolve it:
            tmpGrounding = fg.formula.ground(fg.mrf, {}, allowPartialGroundings=True)
            tmpGrounding.weight = fg.formula.weight
            fg.formula = tmpGrounding
            truth = fg.formula.isTrue(fg.mrf.evidence)
            if truth is not None:
                self.costs -= fg.costs
                print 'processing artifact:', fg, fg.depth, self.variable_stack[fg.depth]
                self.var2fgs[self.variable_stack[fg.depth]].remove(fg)
                fg.parent.children.remove(fg) # this is just for the visualization/ no real functionality
                if fg.depth == min_depth or min_depth is None:
                    min_depth_fgs.append(fg)
                    min_depth = fg.depth
                if fg.depth < min_depth:
                    min_depth = fg.depth
                    min_depth_fgs = []
                    min_depth_fgs.append(fg)
        for fg in min_depth_fgs:
            # remove all variable values from the domain that
            # have been already processed for this subtree
            for var in self.variable_stack[min_depth+1:]:
                for val in self.values_processed[var]:
                    fg.parent.domains[var].remove(val)
            # add the costs which are aggregated by the root of the subtree 
            if not fg.formula.isTrue(fg.mrf.evidence):
                cost += fg.formula.weight * fg.countGroundings()
        # straighten up the variable stack and formula groundings
        # since they might have become empty
        for var in self.var2fgs.keys():
            if len(self.var2fgs[var]) == 0:
                del self.var2fgs[var]
                self.variable_stack.remove(var)
        for var, value in var_assignments.iteritems():
            # skip the variables with values that have already been processed
            if var in self.values_processed and var_assignments[var] in self.values_processed[var]:
                continue
            if not var in values_processed:
                values_processed[var] = []
            if not var in self.variable_stack:
                self.values_processed[var] = []
                depth = len(self.variable_stack)
            else:
                depth = self.variable_stack.index(var)
            indent = ''
            for _ in range(depth):
                indent += '  '
            print indent+'binding', var,'to', value,'@depth:', depth
            queue = list(self.var2fgs[self.variable_stack[depth - 1]])
            while len(queue) > 0:
                fg = queue.pop()
                # first hinge the new variable grounding to all possible parents,
                # i.e. all FormulaGroundings with depth - 1...
                if fg.depth < depth:
                    vars_and_values = [{var: value}]
                # ...then hinge all previously seen subtrees to the newly created formula groundings...    
                elif fg.depth >= depth and fg.depth < len(self.variable_stack) - 1:
                    vars_and_values = [{self.variable_stack[fg.depth + 1]: v} 
                                   for v in self.values_processed[self.variable_stack[fg.depth + 1]]]
                # ...and finally all variable values that are not part of the subtrees
                # (since they have been removed due to falsity of a formula grounding).
                else:
                    vars_and_values = []
                    for varNotInTree in [v for v in self.values_processed.keys() if v not in self.variable_stack]:
                        values = self.values_processed[varNotInTree]
                        for v in values:
                            vars_and_values.append({varNotInTree: v})
                for var_value in vars_and_values:
                    for var_name, val in var_value.iteritems(): break
                    if not fg.domains.get(var_name, None) or not val in fg.domains[var_name]: continue
                    fg.domains[var_name].remove(val)
                    print var_name+'='+val, 'in', fg
                    gnd_result = fg.ground(var_value)
                    if isinstance(gnd_result, FormulaGrounding):
                        # collect all ground atoms that have been created as 
                        # as artifacts for future evaluation
                        artifactGndAtoms = []
                        gnd_result.formula.getGroundAtoms(artifactGndAtoms)
                        for artGndAtom in artifactGndAtoms:
                            artFgs = self.gndAtom2fgs.get(artGndAtom, None)
                            if artFgs is None:
                                artFgs = []
                                self.gndAtom2fgs[artGndAtom] = artFgs
                            gnd_result.mrf.printEvidence()
                            print 'artifact in', gnd_result
                            artFgs.append(gnd_result)
#                         if len(gnd_result.formula.getVariables(self.mrf)) == 0:
#                             print gnd_result.formula
                        if not var in self.variable_stack:
                            self.variable_stack.append(var)
                        if not var_name in self.var2fgs:
                            self.var2fgs[self.variable_stack[gnd_result.depth]] = []
                        if not var_name in var2fgs:
                            var2fgs[self.variable_stack[gnd_result.depth]] = []
                        self.var2fgs[self.variable_stack[gnd_result.depth]].append(gnd_result)
                        var2fgs[self.variable_stack[gnd_result.depth]].append(gnd_result)
                        queue.append(gnd_result)
                    else: # ...otherwise it's true/false; add its costs and discard it.
                        print indent+'caused %.2f costs' % gnd_result
                        cost += gnd_result
                        
            self.printTree()
            self.values_processed[var].append(value)
            values_processed[var].append(value)
        # update the stacks
        self.costs += cost
        self.costStack.append(cost)
        self.values_processedStack.append(values_processed)
        self.var2fgsStack.append(var2fgs)
    
    def printTree(self):
#         print self.depth2fgs
#         print self.variable_stack
#         print self.values_processed
        
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
            if grammar.isVar(p1):
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
        self._recursive_expand(self.varIdx2GndAtom.keys(), 0.)
        
    def _recursive_expand(self, variables, lowerbound):
        # if we have found a solution, update the global upper bound
        indent = ''
        for _ in range(len(self.varIdx2GndAtom)-len(variables)): indent += '    '
        if len(variables) == 0: # all gndAtoms have been assigned
            if lowerbound <= self.upperbound:
                solution = dict([(str(a), self.mrf.evidence[a.idx]) for a in self.mrf.gndAtoms.values()])
                print indent + 'New solution:', solution
                print indent + 'costs:', lowerbound
                self.upperbound = lowerbound
                self.best_solution = solution
                return 
        if lowerbound >= self.upperbound:
            print indent + 'backtracking for LB=%f >= UB=%f' % (lowerbound, self.upperbound)
            return
        # generate the truth assignments to be tested
        variable = variables[0]
        gndAtoms = self.varIdx2GndAtom[variable]
        truthAssignments = []
        if len(gndAtoms) == 1: # binary variable
            truthAssignments.append({gndAtoms[0]: True})
            truthAssignments.append({gndAtoms[0]: False})
        elif len(gndAtoms) > 1: # mutex constraint
            for trueGndAtom in gndAtoms:
                assignment = dict([(gndAtom, False) for gndAtom in gndAtoms])
                truthAssignments.append(assignment)
                assignment[trueGndAtom] = True
        # test the assignments
        for truthAssignment in truthAssignments:
            print indent+'testing', truthAssignment
            # set the temporary evidence
            self.setEvidence(truthAssignment)
            backtrack = []
            doBacktracking = False
            for gndAtom in truthAssignment:
                costs = .0
                for factory in self.factories:
                    factory.ground(gndAtom)
                    costs += factory.costs
                    backtrack.append(factory) 
                    print indent+'costs:'+str(costs)
                    if costs >= self.upperbound:
                        print indent+'backtracking (C=%.2f >= %.2f=UB)' % (costs, self.upperbound)
                        doBacktracking = True
                        break
                else:
                    self._recursive_expand(variables[1:], costs)
                if doBacktracking: break
            # revoke the groundings of the already grounded factories
            for f in backtrack: 
                f.unground()
            # remove the evidence
            self.neutralizeEvidence(truthAssignment)
    
    def setEvidence(self, assignment):
        for gndAtom in assignment:
            self.mrf._setEvidence(gndAtom.idx, assignment[gndAtom])
            
    def neutralizeEvidence(self, assignment):
        for gndAtom in assignment:
            self.mrf._setEvidence(gndAtom.idx, None)
            
if __name__ == '__main__':
    
    mln = PRACMLN()
    mln.declarePredicate('foo', ['x', 'y'])#, functional=[1])
    mln.declarePredicate('bar', ['y','z'])
    
    f = grammar.parseFormula('foo(?x1,?y1) ^ foo(?x2,?y1) ^ bar(?y3,Y) ^ bar(?y3, ?z2)')
    mln.addFormula(f, 1)
#    mln.addDomainValue('x', 'Z')
    
    db = PRACDatabase(mln)
    db.addGroundAtom('!foo(X, Fred)')
    db.addGroundAtom('!foo(X, Ann)')
    db.addGroundAtom('!bar(Fred, Z)')
    db.addGroundAtom('bar(Bob, Y)')
    
    
    mrf = mln.groundMRF(db, simplify=False)
    
    bnb = BranchAndBound(mrf)
    bnb.search()
    for s in bnb.best_solution:
        print s, ':', bnb.best_solution[s]
      
    exit(0)
    groundingFactories = [GroundingFactory(f, mrf) for f in mrf.formulas]
     
    atoms = list(mrf.gndAtoms.keys())
    shuffle(atoms)
    for f in mrf.formulas: print f
    print mrf.domains
    for atom in atoms:
        print 'grounding with', atom
        for factory in groundingFactories:
            factory.ground(mrf.gndAtoms[atom])
#             factory.printTree()
#             factory.unground()
#             factory.printTree()
