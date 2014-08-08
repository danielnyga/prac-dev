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
from mln.database import readDBFromString, readDBFromFile
from mln import readMLNFromFile
from mln.mln import MLN

from utils import colorize


if __name__ == '__main__':
    print "Running main..."

    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False, action='store_true',
                      help="Starts PRAC object recognition with an interactive GUI tool.")
    parser.add_option("-k", "--knowledgebase", dest="dkb", default='all',
                      help="Knowledge base to be used for inference. Options: all, kitchenware, fruit, misc. Example: pracobjrec -k kitchen 'container with a handle'")
    parser.add_option("-c", "--createkbentry", nargs=2, dest="kbentry", default=False, 
                      help="Creates KBMLN with given name or adds entry to existing KBMLN. Example: pracobjrec -c kitchen cup.n.01 'container with a handle'")
    parser.add_option("-d", "--createkbentryFromDB", nargs=2, dest="kbentrydb", default=False, 
                      help="Creates KBMLN with given name from db file. Example: pracobjrec -d kitchen path/to/dbfile/with/kitchenware/entries.db")
    (options, args) = parser.parse_args()

    interactive = options.interactive
    sentences = args

    dkbname = options.dkb

    if options.kbentry:
        kbname = options.kbentry[0]
        conceptname = options.kbentry[1]

    if options.kbentrydb:
        kbname = options.kbentrydb[0]
        dbfile = options.kbentrydb[1]

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
    elif options.kbentrydb: # create initial DKB from db file
        objRec = prac.getModuleByName('obj_recognition')

        # create dkb
        filepath = os.path.join(objRec.module_path, 'kb', '{}.dkb'.format(kbname))
        dkb = objRec.create_dkb(kbname)
        mln = readMLNFromFile(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../pracmodules/prop_extraction/mln/predicates.mln'), logic='FirstOrderLogic', )
        kbdb = readDBFromFile(mln, dbfile)
        conceptname = ''
        for db in kbdb:
            db.write(sys.stdout,color=True)
            formula = []
            for q in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word,?sense) ^ object(?cluster, ?obj)'):
                if q['?sense'] == 'null': continue
                if q['?prop'] == 'null': continue
                conceptname = q['?obj']
                formula.append('property(?c, {0}, {1}) ^ similar({0}, ?w)'.format(q['?sense'], q['?prop']))
            newformula = ' ^ '.join(formula) # conjunct all properties inferred from input sentence
            f = 'object(?c, {}) <=> {}'.format(conceptname, newformula)

            # several definitions of one concept may be in the kbmln, but it is only listed once
            if conceptname not in dkb.concepts:
                dkb.concepts.append(conceptname)

            dkb.kbmln.addFormula(f, weight=1, hard=False, fixWeight=True)
        objRec.save_dkb(dkb, kbname)

        dkb.kbmln.write(sys.stdout, color=True)
        sys.exit(0)
    elif options.kbentry: # create initial DKB or load and update existing with given concept description
        objRec = prac.getModuleByName('obj_recognition')
        
        # create or load dkb
        filepath = os.path.join(objRec.module_path, 'kb', '{}.dkb'.format(kbname))
        if not os.path.isfile(filepath):
            dkb = objRec.create_dkb(kbname)
        else:
            dkb = objRec.load_dkb(kbname)

        # infer properties from nl sentence
        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))

        # several definitions of one concept may be in the kbmln, but it is only listed once
        if conceptname not in dkb.concepts:
            dkb.concepts.append(conceptname)
            
        # create formula to be added
        formula = []
        dbs = infer.inference_steps[-1].output_dbs
        for db in dbs:
            for q in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                if q['?sense'] == 'null': continue
                if q['?prop'] == 'null': continue
                formula.append('property(?c, {0}, {1}) ^ similar({0}, ?w)'.format(q['?sense'], q['?prop']))
        newformula = ' ^ '.join(formula) # conjunct all properties inferred from input sentence
        f = 'object(?c, {}) <=> {}'.format(conceptname, newformula)
        
        dkb.kbmln.addFormula(f, weight=1, hard=False, fixWeight=True)
        objRec.save_dkb(dkb, kbname)

        dkb.kbmln.write(sys.stdout, color=True)
    else: # regular PRAC pipeline
        # property inference from parsed input
        propExtract = prac.getModuleByName('prop_extraction')
        prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))

        # object inference based on inferred properties
        objRec = prac.getModuleByName('obj_recognition')
        prac.run(infer,objRec,kb=objRec.load_pracmt('obj_recog'),dkb=objRec.load_dkb(dkbname))


    step = infer.inference_steps[-1]
    print
    print colorize('+========================+',  (None, 'green', True), True)
    print colorize('| PRAC INFERENCE RESULTS |',  (None, 'green', True), True)
    print colorize('+========================+',  (None, 'green', True), True)
    print
    print 'Object description: {}'.format(colorize(''.join(sentences),  (None, 'green', True), True))
    print
    if step is not None:
        for db in step.output_dbs:
            for q in db.query('object(?cluster, ?object)'):
                if q['?object'] is 'null': continue
                print 'Inferred object: {}'.format(colorize(q['?object'],  (None, 'green', True), True))
            print
        
