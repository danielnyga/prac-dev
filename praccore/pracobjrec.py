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

import sys,os
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
    parser.add_option("-k", "--knowledgebase", dest="dkb", default='all',
                      help="Knowledge base to be used for inference. Options: all, kitchenware, fruit, misc")
    parser.add_option("-c", "--createkbentry", nargs=2, dest="kbentry", default=False, 
                      help="Creates KBMLN with given name or adds entry to existing KBMLN. Example: pracobjrec -c kitchen cup.n.01 'container with a handle'")
    (options, args) = parser.parse_args()

    interactive = options.interactive
    sentences = args

    dkbname = options.dkb

    if options.kbentry:
        kbname = options.kbentry[0]
        conceptname = options.kbentry[1]

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
    elif options.kbentry: # create initial DKB or load and update existing
        objRec = prac.getModuleByName('obj_recognition')
        
        # create or load dkb
        filepath = os.path.join(objRec.module_path, 'mln', '{}.mln'.format(kbname))
        if not os.path.isfile(filepath):
            dkb = objRec.create_dkb(kbname)
        else:
            dkb = objRec.load_dkb(kbname)


        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))

        if conceptname not in dkb.concepts:
            dkb.concepts.append(conceptname)
            
            # create formula to be added
            formula = []
            dbs = infer.inference_steps[-1].output_dbs
            for db in dbs:
                for q in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                    if q['?sense'] == 'null': continue
                    if q['?prop'] == 'null': continue
                    formula.append('property({}, {}, {})'.format(q['?cluster'], q['?word'].split('-')[0], q['?prop']))
            newformula = ' ^ '.join(formula) # conjunct all properties inferred from input sentence
            formula = 'object(?c, {}) <=> '.format(conceptname) + newformula
            
            dkb.kbmln.addFormula(formula, weight=1, hard=False, fixWeight=True)
            objRec.save_dkb(dkb, kbname)

            sys.exit(0)
        else: # todo: maybe overwrite formula?
            pass
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
        
