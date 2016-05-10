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

import sys, os
from optparse import OptionParser
from core.base import PRAC
from prac.pracquery import PRACQueryGUI
from pracmln import MLN
from pracmln.mln.util import colorize
from pracmln.praclog import logger
from core.inference import PRACInference
from core.learning import PRACLearning
from core.wordnet import WordNet


log = logger(__name__)

parser = OptionParser()
parser.add_option("-i", "--interactive", dest="interactive", default=False,
                  action='store_true',
                  help="Starts PRAC object recognition with an interactive "
                       "GUI tool.")
parser.add_option("-t", "--train", dest="trainMLN", nargs=1, default=None,
                  help="Train given MLN with inference results from argument. "
                       "Example: pracobjrec -t orange.n.01 'It is a yellow "
                       "or orange fruit.'")
parser.add_option("-r", "--regular", dest="regular", default=False,
                  action='store_true', help="Runs regular inference pipeline. "
                                            "Arguments: mlnName")
parser.add_option("-f", "--onthefly", dest="onthefly", default=False,
                  action='store_true', help="Generate MLN on the fly")
parser.add_option("-m", "--mln", nargs=2, dest='mln', default=None,
                  help="Runs regular inference pipeline. Arguments: mlnName")

if __name__ == '__main__':
    print "Running main..."

    (options, args) = parser.parse_args()

    interactive = options.interactive
    regular = options.regular
    sentences = args

    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)
    infer = PRACInference(prac, sentences)


    # in case we have natural-language parameters, parse them
    if len(infer.instructions) > 0:
        parser = prac.getModuleByName('nl_parsing')
        prac.run(infer, parser)

    if interactive:  # use the GUI
        log.info('Entering interactive mode')
        gui = PRACQueryGUI(infer)
        gui.open()
    elif options.trainMLN:  # training with property inference output
        log.info('Training MLN {} with result from property inference'.format(
            options.trainMLN))

        # property inference from parsed input
        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer, propExtract)

        objRecog = prac.getModuleByName('obj_recognition')

        praclearn = PRACLearning(prac)
        praclearn.otherParams['mln'] = options.mln[0]
        praclearn.otherParams['logic'] = options.mln[1]
        praclearn.otherParams['concept'] = options.trainMLN
        praclearn.otherParams['onthefly'] = options.onthefly
        praclearn.training_dbs = infer.inference_steps[-1].output_dbs

        objRecog.train(praclearn)
        sys.exit(0)

    else:  # regular PRAC pipeline
        log.info('Entering regular inference pipeline')

        # property inference from parsed input
        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer, propExtract)

        objRecog = prac.getModuleByName('obj_recognition')

        # object inference based on inferred properties
        prac.run(infer, objRecog)

    step = infer.inference_steps[-1]
    print
    print colorize('+========================+', (None, 'green', True), True)
    print colorize('| PRAC INFERENCE RESULTS |', (None, 'green', True), True)
    print colorize('+========================+', (None, 'green', True), True)
    print
    print 'Object description: {}'.format(colorize(''.join(sentences),
                                                   (None, 'white', True),
                                                   True))
    print
    for db in step.output_dbs:
        print 'Inferred properties:'
        for ek in sorted(db.evidence):
            e = db.evidence[ek]
            if e == 1.0 and any(ek.startswith(p) for p in
                                ['color', 'size', 'shape', 'hypernym', 'hasa',
                                 'dimension', 'consistency', 'material']):
                print '{}({}, {}'.format(
                    colorize(ek.split('(')[0], (None, 'white', True), True),
                    colorize(ek.split('(')[1].split(',')[0],
                             (None, 'magenta', True), True),
                    colorize(ek.split('(')[1].split(',')[1],
                             (None, 'green', True), True))

    for db in step.output_dbs:
        print
        print 'Inferred possible concepts:'
        for ek in sorted(db.evidence, key=db.evidence.get, reverse=True):
            e = db.evidence[ek]
            if e > 0.001 and ek.startswith('object'):
                print '{} {}({}, {})'.format(
                    colorize('{:.4f}'.format(e), (None, 'cyan', True), True),
                    colorize('object', (None, 'white', True), True),
                    colorize(ek.split(',')[0].split('(')[1],
                             (None, 'magenta', True), True),
                    colorize(ek.split(',')[1].split(')')[0],
                             (None, 'yellow', True), True))
