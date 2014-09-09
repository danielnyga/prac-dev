# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2013 by Mareike Picklum (mareikep@cs.tum.edu)
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

from prac.core import PRACModule, PRACKnowledgeBase, PRACPIPE, DescriptionKnowledgeBase
from mln import readMLNFromFile, readDBFromFile, Database
import logging
from mln.methods import LearningMethods
from prac.wordnet import WordNet
from prac.inference import PRACInferenceStep
import sys, os
from utils import colorize

class NLObjectRecognition(PRACModule):    

    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log = logging.getLogger(self.name)
        log.info('Running {}'.format(self.name))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| PRAC OBJECT RECOGNITION: RECOGNIZING OBJECTS|', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable object based on nl description properties...', (None, 'white', True), True)
        
        if params.get('dkb') is not None:
            dkb = params.get('dkb')
        else:
            dkb = self.load_dkb('fruit')
        log.info('Using DKB: {}'.format(colorize(dkb.name, (None, 'green', True), True)))
        dkb.kbmln.write(sys.stdout, color=True) # todo remove, debugging only

        if params.get('kb', None) is None:
            # load the default arguments
            dbs = pracinference.inference_steps[-1].output_dbs
            kb = self.load_pracmt('default')
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs
        mln = dkb.kbmln

        inf_step = PRACInferenceStep(pracinference, self)
        wordnet_module = self.prac.getModuleByName('wn_senses')
        
        result_dbs = []
        # process databases
        for db in kb.dbs:
            # adding evidence properties to new query db
            res_db = Database(mln)
            
            propsFound = {}
            for res in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                if res['?prop'] == 'null': continue
                if res['?sense'] == 'null': continue
                if not res['?prop'] in propsFound:
                    propsFound[res['?prop']] = [res['?sense']]
                else:
                    propsFound[res['?prop']].append(res['?sense'])

                gndAtom = 'property({}, {}, {})'.format(res['?cluster'], res['?sense'], res['?prop'])
                res_db.addGroundAtom(gndAtom)

            # for each inferred property, assert all non-matching properties
            for prop in propsFound:
                for word in mln.domains.get('word', []):
                    if not word in propsFound[prop]:
                        gndAtom = '!property({}, {}, {})'.format(res['?cluster'], word, prop)
                        res_db.addGroundAtom(gndAtom)

            # adding word similarities
            res_db = wordnet_module.add_senses_and_similiarities_for_words(res_db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist])
            res_db.write(sys.stdout, color=True)
            
            # infer and update output dbs
            # log.info(kb.query_params)
            # res_db.write(sys.stdout, color=True)
            # inferred_db = mln.infer(evidence_db=res_db, groundingMethod='DefaultGroundingFactory',**kb.query_params)
            inferred_db = mln.infer(evidence_db=res_db, **kb.query_params)
            # print colorize('Inferred DB...', (None, 'green', True), True) 
            # inferred_db.write(sys.stdout,color=True)
            inf_step.output_dbs.extend([inferred_db])

            for q in inferred_db.query('object(?cluster, ?concept)'):
                # print annotations found in result db
                if q['?concept'] == 'null': continue
                log.info('Inferred: object({}, {})'.format(q['?cluster'], colorize(q['?concept'], (None, 'white', True), True)))
        return inf_step


    def train(self, praclearning):
        print colorize('+=================================================================================+', (None, 'green', True), True)
        print colorize('| No training used. Run pracobjrec with according parameters to create DKB files  |', (None, 'green', True), True)
        print colorize('+=================================================================================+', (None, 'green', True), True)


    def createDKB(self, prac, options, infer):
        log = logging.getLogger(self.name)
        possibleProps = ['COLOR','SIZE','SHAPE','HYPERNYM','HASA']
        
        if options.kbentrydb:
            kbname = options.kbentrydb[0]
            dbfile = options.kbentrydb[1]
            
            # create or load dkb
            filepath = os.path.join(self.module_path, 'kb', '{}.dkb'.format(kbname))
            if not os.path.isfile(filepath):
                dkb = self.create_dkb(kbname)
            else:
                dkb = self.load_dkb(kbname)

            mln = readMLNFromFile(os.path.join(self.module_path, '../prop_extraction/mln/predicates.mln'), logic='FuzzyLogic')
            kbdb = readDBFromFile(mln, dbfile)
            conceptname = ''

            # extract properties and word senses appearing in db
            doms = {}
            for db in kbdb:
                for prop in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word,?sense)'):
                    p = prop['?prop']
                    s = prop['?sense']
                    log.info(p)
                    log.info(s)
                    if p == 'null': continue
                    if p in doms:
                        doms[p].add(s)
                    else:
                        doms[p] = set([s])
                log.info(doms)

            # create formulas
            for db in kbdb:            
                # db.write(sys.stdout,color=True)
                evidenceProps = {}
                unknownProps = []
                formulaUnknownProps = []
                formulaEvidenceProps = []
                for q in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word,?sense) ^ object(?cluster, ?obj)'):
                    prop = q['?prop']
                    sense = q['?sense']
                    conceptname = q['?obj']
                    if sense == 'null': continue
                    if prop == 'null': continue

                    if prop in evidenceProps:
                        evidenceProps[prop].append(q['?sense'])
                    else:
                        evidenceProps[prop] = [q['?sense']]

                # if several values for one property have been inferred (e.g. colors yellow and orange),
                # their similarities will be disjuncted 
                i = 0
                log.info(evidenceProps)
                for ep in evidenceProps:
                    if len(evidenceProps[ep]) == 1:
                        formulaEvidenceProps.append('property(?c, ?w{2}, {0}) ^ similar({1}, ?w{2})'.format(ep, evidenceProps[ep][0], i))
                    else:
                        formulaEvidenceProps.append('property(?c, ?w{2}, {0}) ^ ({1})'.format(ep, ' v '.join(['similar({0}, ?w{1})'.format(ep, p) for p in evidenceProps[ep]])), i)
                    i+=1

                unknownProps = set(evidenceProps.keys()).symmetric_difference(set(possibleProps))
                log.info(unknownProps)
                for up in unknownProps:
                    if up in doms:
                        log.info('property(?c, ?w{2}, {0}) ^ ({1})'.format(up, ' v '.join(['similar({0}, ?w{1})'.format(d, i) for d in doms[up]]),i))
                        formulaUnknownProps.append('property(?c, ?w{2}, {0}) ^ ({1})'.format(up, ' v '.join(['similar({0}, ?w{1})'.format(d, i) for d in doms[up]]),i))
                        i+=1

                # conjunct all properties inferred from input sentence as well as unknown properties
                jointFormulaEvidenceProps = ' ^ '.join(formulaEvidenceProps) 
                jointFormulaUnknownProps = '{}'.format(' ^ '.join(formulaUnknownProps))

                log.info(jointFormulaEvidenceProps)
                log.info(jointFormulaUnknownProps)
                if formulaUnknownProps:
                    f = 'object(?c, {0}) ^ {1} ^ ({2})'.format(conceptname, jointFormulaEvidenceProps, jointFormulaUnknownProps)
                else:
                    f = 'object(?c, {}) ^ {}'.format(conceptname, jointFormulaEvidenceProps)

                log.info(f)

                # several definitions of one concept may be in the kbmln, but it is only listed once
                if conceptname not in dkb.concepts:
                    dkb.concepts.append(conceptname)

                dkb.kbmln.addFormula(f, weight=1, hard=False, fixWeight=True)
            self.save_dkb(dkb, kbname)

            dkb.kbmln.write(sys.stdout, color=True)

        elif options.kbentry: # needs update -> will only add inferred properties

            kbname = options.kbentry[0]
            conceptname = options.kbentry[1]

            # create or load dkb
            filepath = os.path.join(self.module_path, 'kb', '{}.dkb'.format(kbname))
            if not os.path.isfile(filepath):
                dkb = self.create_dkb(kbname)
            else:
                dkb = self.load_dkb(kbname)

            # infer properties from nl sentence
            propExtract = prac.getModuleByName('prop_extraction')
            prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))

                
            # create formulaEvidenceProps to be added
            formulaEvidenceProps = []
            dbs = infer.inference_steps[-1].output_dbs
            for db in dbs:
                i = 0
                for q in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                    if q['?sense'] == 'null': continue
                    if q['?prop'] == 'null': continue
                    formulaEvidenceProps.append('property(?c, ?w{2}, {1}) ^ similar({0}, ?w{2})'.format(q['?sense'], q['?prop'],i))
                    i+=1
            

            jointFormulaEvidenceProps = ' ^ '.join(formulaEvidenceProps) # conjunct all properties inferred from input sentence
            f = 'object(?c, {}) ^ {}'.format(conceptname, jointFormulaEvidenceProps)

            dkb.kbmln.addFormula(f, weight=1, hard=False, fixWeight=True)
            
            # several definitions of one concept may be in the kbmln, but it is only listed once
            if conceptname not in dkb.concepts:
                dkb.concepts.append(conceptname)

            self.save_dkb(dkb, kbname)

            dkb.kbmln.write(sys.stdout, color=True)