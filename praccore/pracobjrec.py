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
from utils import colorize


if __name__ == '__main__':
    print "Running main..."

    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False, action='store_true',
                      help="Starts PRAC object recognition with an interactive GUI tool.")
    parser.add_option("-k", "--knowledgebase", dest="dkb", default='fruit',
                      help="Knowledge base to be used for inference. Options: all, kitchenware, fruit, misc")
    parser.add_option("-c", "--createdbs", dest="createdbs", default=False, 
                      help="Creates dkb file with given name. Name must be filename (w\o extension) of existing .db file in db folder of module")
    (options, args) = parser.parse_args()

    interactive = options.interactive
    sentences = args

    dkbname = options.dkb

    log = logging.getLogger()
    log.setLevel(logging.INFO)

    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)
    
    infer = PRACInference(prac, sentences)

    
    
    # in case we have natural-language parameters, parse them
    if len(infer.instructions) > 0:
        parser = prac.getModuleByName('nl_parsing')
        prac.run(infer, parser)
        
    if interactive: # use the GUI
        gui = PRACQueryGUI(infer)
        gui.open()
    elif options.createdbs:
        objRec = prac.getModuleByName('obj_recognition')
        dkb = objRec.create_dkb(options.createdbs)
        objRec.save_dkb(dkb, options.createdbs)
        sys.exit(0)
    else: # regular PRAC pipeline
        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))

        objRec = prac.getModuleByName('obj_recognition')
        prac.run(infer,objRec,kb=objRec.load_pracmt('obj_recog'),dkb=objRec.load_dkb(dkbname))


    step = infer.inference_steps[-1]
    print
    print colorize('+========================+',  (None, 'green', True), True)
    print colorize('| PRAC INFERENCE RESULTS |',  (None, 'green', True), True)
    print colorize('+========================+',  (None, 'green', True), True)
    print
    if step is not None:
        for db in step.output_dbs:
            for ek in sorted(db.evidence.keys()):
                e = db.evidence[ek]
                if e > 0.001 and ek.startswith('object'):
                    print '{0:.2f}    {1}'.format(e, ek)
            print
        
