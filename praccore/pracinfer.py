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
from gui.querytool import PRACQueryGUI
from prac.wordnet import WordNet
from mln.database import readDBFromString

dbs = '''
0.27  is_a(water-2-04, mix.n.01)
0.00  has_sense(Add-1, faucet-5-01)
0.00  has_sense(the-4, water-2-04)
0.00  is_a(Add-1-05, pour.v.01)
0.00  has_sense(water-2, Add-1-06)
0.00  is_a(Add-1-06, pancake.n.01)
0.00  is_a(water-2-02, pour.v.01)
0.00  is_a(faucet-5-01, fill.v.01)
0.00  is_a(Add-1-04, add.v.01)
0.00  is_a(water-2-06, add.v.01)
0.00  is_a(water-2-01, pour.v.01)
0.56  is_a(water-2-04, cooker.n.01)
0.13  is_a(water-2-04, one.n.01)
0.00  is_a(Add-1-02, pour.v.01)
0.00  action_core(water-2, DoingAddition)
0.00  has_sense(water-2, water-2-03)
0.00  has_sense(faucet-5, null)
0.00  is_a(Add-1-02, add.v.01)
0.00  is_a(Add-1-03, flip.v.08)
0.00  is_a(null, pancake.n.01)
0.00  action_core(faucet-5, Flipping)
0.00  has_sense(water-2, null)
0.00  is_a(Add-1-06, cooker.n.01)
0.33  is_a(water-2-02, pancake.n.01)
1.00  action_core(faucet-5, null)
0.00  is_a(water-2-02, flip.v.08)
0.00  is_a(water-2-04, add.v.04)
0.00  is_a(Add-1-04, fill.v.01)
1.00  has_pos(Add-1, VB)
0.00  is_a(Add-1-01, pour.v.01)
0.00  has_sense(faucet-5, water-2-01)
0.00  is_a(Add-1-06, one.n.01)
0.00  is_a(Add-1-04, flip.v.08)
0.00  action_core(Add-1, null)
0.00  is_a(Add-1-02, add.v.04)
0.00  has_sense(the-4, water-2-05)
0.00  is_a(water-2-04, add.v.01)
0.00  has_sense(water-2, Add-1-01)
0.00  is_a(Add-1-03, one.n.01)
0.00  is_a(faucet-5-01, flip.v.08)
0.38  is_a(water-2-05, pancake.n.01)
0.00  is_a(Add-1-04, pour.v.01)
0.43  is_a(water-2-01, mix.n.01)
0.00  has_sense(water-2, water-2-04)
0.00  is_a(null, mix.n.01)
0.38  is_a(water-2-05, mix.n.01)
0.00  is_a(Add-1-04, milk.n.01)
0.00  has_sense(the-4, Add-1-02)
0.43  is_a(water-2-01, pancake.n.01)
0.00  is_a(Add-1-03, add.v.04)
0.00  is_a(Add-1-04, one.n.01)
0.00  is_a(faucet-5-01, add.v.04)
0.55  is_a(faucet-5-01, cooker.n.01)
0.00  is_a(water-2-02, add.v.04)
0.00  has_sense(faucet-5, Add-1-05)
0.00  has_sense(Add-1, water-2-02)
0.00  is_a(water-2-01, fill.v.01)
0.00  is_a(Add-1-06, milk.n.01)
0.00  is_a(water-2-04, flip.v.08)
0.00  has_sense(faucet-5, water-2-02)
1.00  has_sense(Add-1, Add-1-01)
0.43  is_a(water-2-03, mix.n.01)
0.36  is_a(water-2-02, milk.n.01)
0.00  is_a(Add-1-06, pour.v.01)
0.00  has_sense(the-4, Add-1-01)
1.00  is_a(Add-1-01, add.v.01)
0.00  has_sense(the-4, water-2-02)
0.00  has_sense(faucet-5, Add-1-02)
0.00  is_a(null, fill.v.01)
0.00  is_a(Add-1-02, one.n.01)
0.00  has_sense(water-2, water-2-05)
0.24  is_a(water-2-06, cooker.n.01)
0.71  is_a(water-2-06, mix.n.01)
0.00  action_core(Add-1, Flipping)
0.00  is_a(Add-1-04, cooker.n.01)
0.00  has_sense(faucet-5, water-2-03)
0.00  has_sense(the-4, Add-1-03)
0.00  has_sense(faucet-5, Add-1-04)
0.27  is_a(water-2-04, pancake.n.01)
0.00  is_a(water-2-05, add.v.01)
0.00  is_a(water-2-04, fill.v.01)
0.00  is_a(Add-1-06, mix.n.01)
0.00  is_a(Add-1-02, cooker.n.01)
0.00  is_a(Add-1-05, flip.v.08)
0.00  has_sense(Add-1, water-2-05)
0.42  is_a(water-2-01, milk.n.01)
0.00  has_sense(Add-1, Add-1-02)
0.00  is_a(Add-1-05, mix.n.01)
0.25  is_a(water-2-06, one.n.01)
1.00  has_sense(water-2, water-2-06)
0.00  is_a(Add-1-02, milk.n.01)
0.33  is_a(water-2-02, mix.n.01)
0.43  is_a(water-2-03, pancake.n.01)
0.00  is_a(water-2-05, add.v.04)
0.00  is_a(water-2-06, add.v.04)
0.00  has_sense(the-4, water-2-03)
1.00  prep_from(Add-1, faucet-5)
0.29  is_a(water-2-04, milk.n.01)
0.00  action_core(faucet-5, FluidFlowTranslation)
0.00  is_a(faucet-5-01, pour.v.01)
0.00  has_sense(Add-1, Add-1-03)
0.00  has_sense(faucet-5, water-2-04)
0.00  is_a(Add-1-06, add.v.04)
0.00  is_a(Add-1-01, cooker.n.01)
0.00  is_a(Add-1-03, mix.n.01)
0.00  action_core(water-2, Flipping)
0.00  is_a(Add-1-01, fill.v.01)
0.00  has_sense(Add-1, water-2-03)
0.44  is_a(water-2-05, milk.n.01)
1.00  has_sense(the-4, null)
0.11  is_a(faucet-5-01, one.n.01)
0.00  is_a(Add-1-02, flip.v.08)
0.00  has_sense(Add-1, water-2-04)
0.00  has_sense(water-2, faucet-5-01)
0.00  is_a(water-2-05, fill.v.01)
0.00  is_a(Add-1-05, add.v.01)
0.00  is_a(water-2-03, fill.v.01)
0.00  is_a(Add-1-05, milk.n.01)
0.00  is_a(Add-1-02, pancake.n.01)
0.00  is_a(null, milk.n.01)
0.00  is_a(null, one.n.01)
0.17  is_a(water-2-02, one.n.01)
1.00  has_pos(faucet-5, NN)
0.00  is_a(Add-1-01, flip.v.08)
1.00  has_pos(the-4, DT)
0.00  is_a(water-2-01, add.v.04)
0.00  has_sense(faucet-5, Add-1-06)
0.00  has_sense(the-4, Add-1-04)
0.00  is_a(Add-1-01, milk.n.01)
0.00  is_a(Add-1-05, fill.v.01)
0.00  is_a(null, flip.v.08)
0.00  has_sense(faucet-5, water-2-05)
0.00  has_sense(Add-1, Add-1-04)
0.00  is_a(Add-1-03, pancake.n.01)
0.25  is_a(water-2-01, one.n.01)
0.00  is_a(Add-1-05, pancake.n.01)
0.00  has_sense(water-2, Add-1-03)
0.00  action_core(faucet-5, DoingAddition)
0.00  is_a(faucet-5-01, add.v.01)
1.00  det(faucet-5, the-4)
0.00  is_a(Add-1-06, fill.v.01)
0.00  has_sense(water-2, Add-1-05)
0.00  is_a(water-2-06, flip.v.08)
0.00  has_sense(the-4, faucet-5-01)
1.00  dobj(Add-1, water-2)
0.00  is_a(Add-1-01, mix.n.01)
0.00  is_a(Add-1-03, add.v.01)
0.00  is_a(Add-1-01, add.v.04)
0.00  has_sense(faucet-5, Add-1-01)
0.00  is_a(water-2-03, add.v.01)
0.00  has_sense(the-4, Add-1-06)
0.00  is_a(Add-1-01, pancake.n.01)
1.00  is_a(Add-1-04, add.v.04)
0.00  has_sense(Add-1, water-2-06)
0.00  is_a(null, cooker.n.01)
0.00  has_sense(Add-1, Add-1-05)
0.00  is_a(Add-1-05, one.n.01)
0.00  action_core(the-4, FluidFlowTranslation)
0.00  has_sense(faucet-5, water-2-06)
0.24  is_a(water-2-01, cooker.n.01)
0.82  is_a(water-2-06, milk.n.01)
0.00  is_a(water-2-03, flip.v.08)
0.00  has_sense(water-2, Add-1-02)
0.43  is_a(water-2-06, pancake.n.01)
0.00  has_sense(water-2, Add-1-04)
0.00  has_sense(the-4, water-2-06)
0.00  is_a(water-2-02, add.v.01)
0.00  is_a(water-2-06, fill.v.01)
0.00  is_a(water-2-03, pour.v.01)
0.00  is_a(water-2-02, fill.v.01)
1.00  has_sense(faucet-5, faucet-5-01)
0.00  has_sense(the-4, Add-1-05)
0.00  has_sense(water-2, water-2-01)
0.00  is_a(Add-1-04, mix.n.01)
0.24  is_a(water-2-03, cooker.n.01)
0.00  is_a(Add-1-06, add.v.01)
0.00  is_a(water-2-05, flip.v.08)
0.00  has_sense(Add-1, water-2-01)
0.24  is_a(water-2-05, one.n.01)
0.21  is_a(faucet-5-01, pancake.n.01)
0.22  is_a(faucet-5-01, milk.n.01)
0.00  has_sense(Add-1, Add-1-06)
0.00  action_core(water-2, FluidFlowTranslation)
0.00  has_sense(Add-1, null)
0.00  is_a(null, pour.v.01)
0.00  is_a(Add-1-03, milk.n.01)
0.00  is_a(water-2-06, pour.v.01)
0.27  is_a(water-2-03, one.n.01)
0.00  is_a(Add-1-06, flip.v.08)
0.21  is_a(water-2-05, cooker.n.01)
0.00  is_a(null, add.v.04)
0.00  has_sense(the-4, water-2-01)
0.00  action_core(the-4, DoingAddition)
1.00  action_core(the-4, null)
1.00  root(ROOT-0, Add-1)
1.00  action_core(Add-1, FluidFlowTranslation)
1.00  has_pos(water-2, NN)
0.00  is_a(Add-1-05, cooker.n.01)
0.21  is_a(faucet-5-01, mix.n.01)
0.00  is_a(water-2-04, pour.v.01)
0.00  is_a(Add-1-01, one.n.01)
0.67  is_a(Add-1-03, fill.v.01)
0.50  is_a(water-2-03, milk.n.01)
0.00  has_sense(water-2, water-2-02)
0.00  is_a(Add-1-05, add.v.04)
0.00  is_a(null, add.v.01)
1.00  action_core(water-2, null)
0.00  is_a(Add-1-03, cooker.n.01)
0.00  is_a(water-2-01, add.v.01)
0.00  has_sense(faucet-5, Add-1-03)
0.00  is_a(Add-1-04, pancake.n.01)
0.00  is_a(water-2-01, flip.v.08)
0.00  action_core(Add-1, DoingAddition)
0.00  is_a(Add-1-02, fill.v.01)
0.27  is_a(water-2-02, cooker.n.01)
0.00  is_a(water-2-05, pour.v.01)
0.00  is_a(Add-1-03, pour.v.01)
0.00  is_a(water-2-03, add.v.04)
0.00  action_core(the-4, Flipping)
0.00  is_a(Add-1-02, mix.n.01)
'''
parser = OptionParser()
parser.add_option("-i", "--interactive", dest="interactive", default=False, action='store_true',
                  help="Starts PRAC inference with an interactive GUI tool.")

if __name__ == '__main__':
    
    (options, args) = parser.parse_args()
    
    interactive = options.interactive
    
    sentences = args
    
    log = logging.getLogger()
#     log.setLevel(logging.DEBUG)

    prac = PRAC()
#     actionCore = prac.getModuleByName('ac_recognition')
    prac.wordnet = WordNet(concepts=None)
    
    infer = PRACInference(prac, sentences)
#     actionCore.insertdbs(infer, *readDBFromString(prac.mln, dbs))
    
    # in case we have natural-language parameters, parse them
    if len(infer.instructions) > 0:
        parser = prac.getModuleByName('nl_parsing')
        prac.run(infer, parser)
        
    if interactive: # use the GUI
        gui = PRACQueryGUI(infer)
        gui.open()
    else: # regular PRAC pipeline
        # get the action cores activated
        actionCore = prac.getModuleByName('ac_recognition')
        prac.run(infer,actionCore,kb=actionCore.load_pracmt('robohow-new'))
        
        # assign the roles to the words
        actionRoles = prac.getModuleByName('senses_and_roles')
        prac.run(infer,actionRoles)
#         
#         # infer the missing roles
        prac.run(infer, actionRoles, missing=True)    
        
#     
    step = infer.inference_steps[-1]
    print '======================'
    print 'PRAC INFERENCE RESULTS'
    print '======================'
    for db in step.output_dbs:
        for a in sorted(db.evidence.keys()):
            v = db.evidence[a]
            if v > 0.001 and (a.startswith('action_core') or a.startswith('action_role') or a.startswith('has_sense')):
                print '%.3f    %s' % (v, a)
        print '---'
    
    
            
