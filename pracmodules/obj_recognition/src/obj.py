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

possibleProps = ['COLOR','SIZE','SHAPE','HYPERNYM','HASA']

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
            dkb = self.load_dkb('mini')
        dkb.printDKB()
        mln = dkb.kbmln

        if params.get('kb', None) is None:
            # load the default arguments
            kb = self.load_pracmt('obj_recog')
            dbs = pracinference.inference_steps[-1].output_dbs
            kb.dbs = dbs
        else:
            kb = params['kb']
        if not hasattr(kb, 'dbs'):
            kb.dbs = pracinference.inference_steps[-1].output_dbs

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
            # for prop in propsFound:
            #     for word in mln.domains.get('word', []):
            #         if not word in propsFound[prop]:
            #             gndAtom = '!property({}, {}, {})'.format(res['?cluster'], word, prop)
            #             res_db.addGroundAtom(gndAtom)

            # for each NOT inferred property, assume it has value 'Unknown' (which has a similarity of 0.01 to everything)
            # todo: check if this makes much of a difference
            # for prop in set(propsFound.keys()).symmetric_difference(set(possibleProps)):
            #     gndAtom = 'property({}, {}, {})'.format(res['?cluster'], 'Unknown', prop)
            #     res_db.addGroundAtom(gndAtom)

            # adding word similarities
            res_db = wordnet_module.add_senses_and_similiarities_for_words(res_db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist])
            # res_db = wordnet_module.add_senses_and_similiarities_for_words(res_db, mln.domains.get('word', []) + [item for sublist in propsFound.values() for item in sublist] + ['Unknown'])
            # res_db.write(sys.stdout, color=True)
            
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
        print colorize('+===============================================+', (None, 'green', True), True)
        print colorize('| CREATING OR MODIFYING DKB FILE                +', (None, 'green', True), True)
        print colorize('+===============================================+', (None, 'green', True), True)

        log = logging.getLogger(self.name)
        log.info(options)

        if options.kbentrydb: # creates or updates DKB with database contents
            kbName = options.kbentrydb[0]
            dbfile = options.kbentrydb[1]
            log.info('Creating or modifying \'{}\'...     '.format(colorize(kbName + '.dkb',(None, 'green', True), True)))
            
            # create or load dkb
            filepath = os.path.join(self.module_path, 'kb', '{}.dkb'.format(kbName))
            if not os.path.isfile(filepath):
                dkb = self.create_dkb(kbName)
            else:
                dkb = self.load_dkb(kbName)

            mln = readMLNFromFile(os.path.join(self.module_path, '../prop_extraction/mln/predicates.mln'), logic='FuzzyLogic')
            kbdb = readDBFromFile(mln, dbfile)
            conceptname = ''
            dkbDict = dkb.conceptDict
            dkb.kbmln.formulas = []

            # create formulas
            for concept in kbdb:            
                # db.write(sys.stdout,color=True)
                evidenceProps = {}
                unknownProps = []
                for q in concept.query('property(?cluster, ?word, ?prop) ^ has_sense(?word,?sense) ^ object(?cluster, ?obj)'):
                    prop = q['?prop']
                    sense = q['?sense']
                    conceptname = q['?obj']
                    if sense == 'null': continue
                    if prop == 'null': continue

                    if prop in evidenceProps:
                        evidenceProps[prop].append(sense)
                    else:
                        evidenceProps[prop] = [sense]
                
                # make sure entries don't get overwritten
                self.updateDKBDict(conceptname, dkbDict, evidenceProps, False)


        elif options.kbentry: # creates or updates DKB with inferred results from params

            kbName = options.kbentry[0]
            conceptname = options.kbentry[1]
            log.info('Creating or modifying \'{}\'...     '.format(colorize(kbName + '.dkb',(None, 'green', True), True)))

            # create or load dkb
            filepath = os.path.join(self.module_path, 'kb', '{}.dkb'.format(kbName))
            if not os.path.isfile(filepath):
                dkb = self.create_dkb(kbName)
            else:
                dkb = self.load_dkb(kbName)

            dkbDict = dkb.conceptDict
            dkb.kbmln.formulas = []

            # infer properties from nl sentence
            propExtract = prac.getModuleByName('prop_extraction')
            prac.run(infer,propExtract,kb=propExtract.load_pracmt('prop_extract'))

                
            # create formulaEvidenceProps to be added
            formulaEvidenceProps = []
            dbs = infer.inference_steps[-1].output_dbs
            evidenceProps = {}
            for db in dbs:
                i = 0
                for q in db.query('property(?cluster, ?word, ?prop) ^ has_sense(?word, ?sense)'):
                    sense = q['?sense']
                    prop = q['?prop']
                    if sense == 'null': continue
                    if prop == 'null': continue
                    formulaEvidenceProps.append('property(?c, ?w{2}, {1}) ^ similar({0}, ?w{2})'.format(sense, prop,i))
                    i+=1

                    if prop in evidenceProps:
                        evidenceProps[prop].append(sense)
                    else:
                        evidenceProps[prop] = [sense]

                # make sure entries don't get overwritten
                self.updateDKBDict(conceptname, dkbDict, evidenceProps, False)
            
        # any case:
        doms = {}
        for concept in dkbDict:
            for prop in dkbDict[concept]:
                if prop in doms:
                    doms[prop] = list(set(doms[prop] + dkbDict[concept][prop]))
                else:
                    doms[prop] = dkbDict[concept][prop]

        log.info(doms)
        for conceptname in dkbDict:
            i = 0
            props = dkbDict[conceptname]
            formulaUnknownProps = []
            formulaEvidenceProps = []            
            for ep in props:
                if len(props[ep]) == 1:
                    formulaEvidenceProps.append('property(?c, ?w{2}, {0}) ^ similar({1}, ?w{2})'.format(ep, props[ep][0], i))
                else:
                    simsE = ' v '.join(['similar({0}, ?w{1})'.format(p, i) for p in props[ep]])
                    formulaEvidenceProps.append('property(?c, ?w{2}, {0}) ^ ({1})'.format(ep, simsE, i))
                i+=1

            # all properties that have NOT been inferred are added to the formula with their values being disjunctions
            # of all possible assignments
            unknownProps = set(props.keys()).symmetric_difference(set(possibleProps))
            for up in unknownProps:
                if up in doms:
                    simsU = ' v '.join(['similar({0}, ?w{1})'.format(d, i) for d in doms[up]])
                    formulaUnknownProps.append('property(?c, ?w{2}, {0}) ^ ({1})'.format(up, simsU, i))
                    i+=1

            # conjunct all properties inferred from input sentence as well as unknown properties
            jointFormulaEvidenceProps = ' ^ '.join(formulaEvidenceProps) 
            jointFormulaUnknownProps = '{}'.format(' ^ '.join(formulaUnknownProps))

            if False:
                f = 'object(?c, {0}) ^ {1} ^ ({2})'.format(conceptname, jointFormulaEvidenceProps, jointFormulaUnknownProps)
            else:
                f = 'object(?c, {}) ^ {}'.format(conceptname, jointFormulaEvidenceProps)

            dkb.kbmln.addFormula(f, weight=1, hard=False, fixWeight=True)
        dkb.conceptDict = dkbDict
        self.save_dkb(dkb, kbName)

        dkb.printDKB()

    
    def updateDKBDict(self, conceptname, dct, newprops, overwrite):
            if conceptname in dct:
                prps = dct[conceptname]
                if overwrite:
                    dct[conceptname] = dict(prps.items() + newprops.items())
                else:
                    for x in prps:
                        for y in newprops:
                            if x == y:
                                l = list(set(prps[x]).union(set(newprops[y])))
                                dct[conceptname][x] = l
            else:               
                dct[conceptname] = newprops