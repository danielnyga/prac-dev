# PROBABILISTIC ROBOT ACTION CORES - LEARNING
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

import sys
from optparse import OptionParser
from prac.core import PRAC
from praclog import logging
from prac.inference import PRACInference

dbs = '''
// Stir everything with a spoon .
has_sense(spoon-5,spoon-5-sense)
is_a(spoon-5-sense,spoon.n.01)
root(ROOT-0, Stir-1)
dobj(Stir-1, everything-2)
det(spoon-5, a-4)
prep_with(Stir-1, spoon-5)
has_pos(Stir-1,VB)
has_pos(everything-2,NN)
has_pos(a-4,DT)
has_pos(spoon-5,NN)
has_sense(Stir-1, Stir-1-sense)
has_sense(everything-2, null)
has_sense(a-4, null)
is_a(Stir-1-sense, stir.v.01)
---
// fill the pot up with water .
has_sense(fill-1,fill-1-sense)
is_a(fill-1-sense,fill.v.01)
has_sense(pot-3,pot-3-sense)
is_a(pot-3-sense,pot.n.01)
has_sense(up-4,up-4-sense)
is_a(up-4-sense,up.r.01)
has_sense(water-6,water-6-sense)
is_a(water-6-sense,water.n.06)
root(ROOT-0, fill-1)
det(pot-3, the-2)
npadvmod(up-4, pot-3)
advmod(fill-1, up-4)
prep_with(fill-1, water-6)
has_pos(fill-1,VB)
has_pos(the-2,DT)
has_pos(pot-3,NN)
has_pos(up-4,RB)
has_pos(water-6,NN)
has_sense(the-2, null)
'''

if __name__ == '__main__':
    
    parser = OptionParser()
    (options, args) = parser.parse_args()
    
    sentences = ["Pour the mix into the pan.", "Flip the pancake around."]
    
    log = logging.getLogger()
    log.setLevel(logging.INFO)

    prac = PRAC()
    parsing = prac.getModuleByName('nl_parsing')
    actioncore_recognititon = prac.getModuleByName('ac_recognition')
    
    inf = PRACInference(prac, sentences)

    # do the NL parsing
    prac.run(inf, parsing)
    prac.run(inf, parsing.dbfromstring, dbs)
    
    # add all possible word meanings
#     prac.run(inf, actioncore_recognititon, )
#     
    step = inf.inference_steps[-1]
    for db in step.output_dbs:
        db.write(sys.stdout, color=True)
        print '---'
    
    
            
