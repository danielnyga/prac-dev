# PROBABILISTIC ROBOT ACTION CORES - LEARNING
#
# (C) 2014 by Mareike Picklum (mareikep@cs.uni-bremen.de)
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

import sys,os
from optparse import OptionParser
from prac.core import PRAC
from praclog import logging
from prac.inference import PRACInference
from prac.learning import PRACLearning
from gui.querytool import PRACQueryGUI
from prac.wordnet import WordNet
from mln.database import readDBFromString, readDBFromFile
from mln import readMLNFromFile
from mln.mln import MLN

from utils import colorize


if __name__ == '__main__':
    print "Running main..."
    log = logging.getLogger()
    log.setLevel(logging.INFO)

    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False, action='store_true',
                      help="Starts PRAC object recognition with an interactive GUI tool.")
    parser.add_option("-o", "--useOld", dest="useOld", default=False, action='store_true',
                      help="Uses property(x,y,{COLOR,SIZE,HYPERNYM...}) instead of color(x,y), size(x,y)...")
    parser.add_option("-t", "--train", nargs=2, dest='trainDKB', default=None,
                      help="Train given DKB with inference results from argument. Example: pracobjrec -t fruit orange.n.01 'It is a yellow or orange fruit.'")    
    parser.add_option("-s", "--showDKB", nargs=1, dest='showDKB', default=False, 
                      help="Prints content of given DKB and exits.")    
    parser.add_option("-r", "--regular", nargs=1, dest='dkbName', default='sherlockNew', 
                      help="Runs regular inference pipeline. Arguments: dkbName")    

    (options, args) = parser.parse_args()

    interactive = options.interactive
    sentences = args

    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)
    
    if options.showDKB: # print content of given dkb file and exit
        objRecog = prac.getModuleByName('obj_recognition')
        dkb=objRecog.load_dkb(options.showDKB)
        dkb.printDKB()
        sys.exit(0)

    infer = PRACInference(prac, sentences)
    
    
    # in case we have natural-language parameters, parse them
    if len(infer.instructions) > 0:
        parser = prac.getModuleByName('nl_parsing')
        prac.run(infer, parser)
        
    if interactive: # use the GUI
        log.info('Entering interactive mode')
        gui = PRACQueryGUI(infer)
        gui.open()
    elif options.trainDKB: # training with property inference output
        log.info('Training DKB {} with result from property inference'.format(options.trainDKB[0]))

        # property inference from parsed input
        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))

        if options.useOld:
            objRecog = prac.getModuleByName('obj_recognition_old')
        else:
            objRecog = prac.getModuleByName('obj_recognition')
        
        praclearn = PRACLearning(prac)
        praclearn.otherParams['kb'] = options.trainDKB[0]
        praclearn.otherParams['concept'] = options.trainDKB[1]
        praclearn.otherParams['useOld'] = options.useOld
        praclearn.training_dbs = infer.inference_steps[-1].output_dbs

        objRecog.train(praclearn)
        sys.exit(0)

    else: # regular PRAC pipeline
        log.info('Entering regular inference pipeline')

        # property inference from parsed input
        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))
        
        if options.useOld:
            objRecog = prac.getModuleByName('obj_recognition_old')
        else:
            objRecog = prac.getModuleByName('obj_recognition')

        # object inference based on inferred properties
        prac.run(infer,objRecog,kb=objRecog.load_pracmt('default'),dkb=objRecog.load_dkb(options.dkbName))

    step = infer.inference_steps[-1]
    print
    print colorize('+========================+',  (None, 'green', True), True)
    print colorize('| PRAC INFERENCE RESULTS |',  (None, 'green', True), True)
    print colorize('+========================+',  (None, 'green', True), True)
    print
    print 'Object description: {}'.format(colorize(''.join(sentences),  (None, 'white', True), True))
    print
    for db in step.output_dbs:
        print 'Inferred properties:'
        for ek in sorted(db.evidence):
            e = db.evidence[ek]
            if e == 1.0 and any(ek.startswith(p) for p in ['color','size','shape','isA','hasA']):
                print '{}({}, {}'.format(  colorize(ek.split('(')[0], (None, 'white', True), True), # propertytype
                                            colorize(ek.split('(')[1].split(',')[0], (None, 'magenta', True), True), # cluster
                                            colorize(ek.split('(')[1].split(',')[1], (None, 'green', True), True)) # propertyvalue (wn concept)

    for db in step.output_dbs:
        print
        print 'Inferred possible concepts:'
        for ek in sorted(db.evidence, key=db.evidence.get, reverse=True):
            e = db.evidence[ek]
            if e > 0.001 and ek.startswith('object'):
                print '{} {}({}, {})'.format(   colorize('{:.4f}'.format(e),  (None, 'cyan', True), True), # probability
                                                colorize('object',  (None, 'white', True), True), # 'object'
                                                colorize(ek.split(',')[0].split('(')[1], (None, 'magenta', True), True), # cluster
                                                colorize(ek.split(',')[1].split(')')[0], (None, 'yellow', True), True)) # objectvalue (wn concept)