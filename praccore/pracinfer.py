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
from mln.database import readDBFromString,Database
from utils import colorize
import re
from pracutils.RolequeryHandler import RolequeryHandler

parser = OptionParser()
parser.add_option("-i", "--interactive", dest="interactive", default=False, action='store_true',
                  help="Starts PRAC inference with an interactive GUI tool.")

                
if __name__ == '__main__':
    
    (options, args) = parser.parse_args()
    
    interactive = options.interactive
    
    sentences = args
    
    log = logging.getLogger()
    log.setLevel(logging.ERROR)
    actionRoles = None
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)
    
    infer = PRACInference(prac, sentences)
#     actionCore.insertdbs(infer, *readDBFromString(prac.mln, dbs))
    
    if interactive: # use the GUI
        # in case we have natural-language parameters, parse them
        if len(infer.instructions) > 0:
            parser = prac.getModuleByName('nl_parsing')
            prac.run(infer, parser)
        gui = PRACQueryGUI(infer)
        gui.open()
    else: # regular PRAC pipeline
        while infer.next_module() != None :
            module = prac.getModuleByName(infer.next_module())
            prac.run(infer,module)
    
    step = infer.inference_steps[-1]
    print
    print colorize('+========================+',  (None, 'green', True), True)
    print colorize('| PRAC INFERENCE RESULTS |',  (None, 'green', True), True)
    print colorize('+========================+',  (None, 'green', True), True)
    wordnet_module = prac.getModuleByName('wn_senses')
    for db in step.output_dbs:
        for a in sorted(db.evidence.keys()):
            v = db.evidence[a]
            if v > 0.001 and (a.startswith('action_core') or a.startswith('has_sense') or a.startswith('achieved_by')):
                if a.startswith('has_sense'): 
                    
                    group = re.split(',',re.split('has_sense\w*\(|\)',a)[1])
                    word = group[0];
                    sense = group[1];
                    if sense != 'null':
                        print
                        print colorize('  WORD:', (None, 'white', True), True), word, 
                        print colorize('  SENSE:', (None, 'white', True), True), sense
                        wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(db, word), sense)
                        print
                    else:
                        print '%.3f    %s' % (v, a)
                else:
                    print '%.3f    %s' % (v, a)
        
        RolequeryHandler.queryRolesBasedOnAchievedBy(db).printEvidence()
        print '---'
    
        
    
    
            
