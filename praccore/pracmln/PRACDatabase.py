# MARKOV LOGIC NETWORKS FOR PROBABILISTIC ROBOT ACTION CORES
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

from mln.database import Database
from logic.fol import Lit
from logic.grammar import parseFormula
#from pyswip.prolog import Prolog

class PRACDatabase(Database):
    '''
    Database implementation for PRAC with some convenience methods
    for dynamically generating and manipulating MLN Databases at runtime.
    '''
    def __init__(self, mln, db=None):
        if isinstance(db, Database):
            super(PRACDatabase, self).__init__(mln)
            self.domains = dict(db.domains)
            self.evidence = dict(db.evidence)
            self.softEvidence = list(db.softEvidence)
        else:
            super(PRACDatabase, self).__init__(mln)
    
    def addGroundAtom(self, gndAtom):
        '''
        Adds the fact represented by the ground atom string.
        The domains in the associated MLN instance are updated 
        accordingly, if necessary. 
        '''
        f = parseFormula(gndAtom)
        assert isinstance(f, Lit)
        predName = f.predName
        params = f.params
        isTrue = not f.negated
        atomString = "%s(%s)" % (predName, ",".join(params))
        self.evidence[atomString] = isTrue
        domNames = self.mln.predicates[predName]
        for i, v in enumerate(params):
            if domNames[i] not in self.domains:
                self.domains[domNames[i]] = []
            d = self.domains[domNames[i]]
            if v not in d:
                d.append(v)
                
    def query(self, formula):
        '''
        Makes to the database a 'prolog-like' query given by the specified formula.
        Returns a dictionary with variable-value assignments for which the formula is true.
        ''' 
        pseudoMRF = Database.PseudoMRF(self)
        formula = parseFormula(formula)
        for varAssignment in pseudoMRF.iterTrueVariableAssignments(formula):
            yield varAssignment

def countTrueGroundings(dbs, f):
    '''
    Counts the true groundings of a formula f in a list of Database objects.
    - dbs:    list of Database objects
    - f:      formula of which the groundings shall be counted. Formula object or string.
    '''
    if type(f) is str:
        f = parseFormula(f)
    pseudoMRFs = [db.getPseudoMRF() for db in dbs]
    numTrue = 0
    numTotal = 0
    for db, mrf in zip(dbs,pseudoMRFs):
        dbNumTrue, dbNumTotal = mrf.countTrueGroundings(f)
        numTrue += dbNumTrue
        numTotal += dbNumTotal
    return numTrue, numTotal