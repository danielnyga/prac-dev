# PROBABILISTIC ROBOT ACTION CORES 
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
import os
import sys
from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from pracmln import MLNQuery, MLN, Database

from pracmln.mln.base import parse_mln
from pracmln.mln.methods import LearningMethods
from pracmln.mln.util import colorize
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject

log = logger(__name__)
possibleProps = ['color', 'size', 'shape', 'hypernym', 'hasa']#, 'dimension', 'consistency', 'material']

class NLObjectRecognition(PRACModule):    


    def initialize(self):
        pass
    
    @PRACPIPE
    def __call__(self, pracinference, **params):
        log.info('Running {}'.format(self.name))
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| PRAC OBJECT RECOGNITION: RECOGNIZING OBJECTS|', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        print
        print colorize('Inferring most probable object based on nl description properties...', (None, 'white', True), True)
        
        if params.get('project', None) is None:
            # load default project
            projectpath = self.project_path
            project = MLNProject.open(projectpath)
        else:
            log.info(colorize('Loading Project from params', (None, 'cyan', True), True))
            projectpath = os.path.join(params.get('projectpath', None) or self.module_path, params.get('project').name)
            project = params.get('project')

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs

        mlntext = project.mlns.get(project.queryconf['mln'], None)
        mln = parse_mln(mlntext, searchpaths=[self.module_path], projectpath=projectpath, logic=project.queryconf.get('logic', 'FuzzyLogic'), grammar=project.queryconf.get('grammar', 'PRACGrammar'))
        wordnet_module = self.prac.getModuleByName('wn_senses')

        # adding evidence properties to new query db
        for db in dbs:
            # find properties and add word similarities
            propsFound = self.processDB(db)
            output_db = wordnet_module.add_similarities(db, mln.domains, propsFound)
            output_db.write(sys.stdout, color=True)
            unified_db = output_db.union(db)

            # infer and update output dbs
            infer = MLNQuery(config=project.queryconf, db=unified_db, mln=mln).run()
            result_db = infer.resultdb

            inf_step.output_dbs.append(result_db)

        return inf_step

    def train(self, praclearning):
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| TRAINING KNOWLEDGEBASE...                   |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)


        mlnName =  praclearning.otherParams.get('mln', None)
        mlnLogic =  praclearning.otherParams.get('logic', None)
        objName = praclearning.otherParams.get('concept', None)
        onTheFly = praclearning.otherParams.get('onthefly', False)

        mln = MLN(mlnfile=os.path.abspath(mlnName), logic=mlnLogic, grammar='PRACGrammar')

        pracTrainingDBS = praclearning.training_dbs
        trainingDBS = []

        if len(pracTrainingDBS) >= 1 and type(pracTrainingDBS[0]) is str: # db from file
            log.info('Learning from db files...')
            inputdbs = Database.load(mln, dbfile=pracTrainingDBS, ignore_unknown_preds=True)
            trainingDBS += inputdbs
        elif len(pracTrainingDBS) > 1:
            log.info('Learning from db files (xfold)...')
            trainingDBS = pracTrainingDBS
        else: # db from inference result
            log.info('Learning from inference result...')
            inputdbs = pracTrainingDBS
            for db in inputdbs:
                db << 'object(cluster, {})'.format(objName)
                trainingDBS.append(db)

        outputfile = '{}_trained.mln'.format(mlnName.split('.')[0])
        if onTheFly: # generating formulas from training examples
            print 'generating on the fly'
            trainedMLN = self.generateMLN(trainingDBS, mln)
        else: # learning mln
            trainedMLN = mln.learnWeights(trainingDBS, LearningMethods.DCLL, evidencePreds=possibleProps, partSize=1, gaussianPriorSigma=10, useMultiCPU=0, optimizer='cg', learningRate=0.9)
        
        print colorize('+=============================================+', (None, 'green', True), True)
        print colorize('| LEARNT FORMULAS:                            |', (None, 'green', True), True)
        print colorize('+=============================================+', (None, 'green', True), True)
        
        trainedMLN.printFormulas()
        trainedMLN.write(file(outputfile, "w"))

        return trainedMLN


    # extract found properties from evidence db. used for adding similarities
    def processDB(self, db):
        propsFound = {}
        for prop in possibleProps:
            for q in db.query('{}(?cluster, ?word)'.format(prop)):
                if q['?word'] == 'null': continue
                if q['?cluster'] == 'null': continue
                word = q['?word']
                cluster = q['?cluster']
                if not prop in propsFound:
                    propsFound[prop] = [word]
                else:
                    propsFound[prop].append(word)
                propsFound['cluster'] = cluster
        return propsFound

    # experimental! Will generate an MLN formulas on the fly - 
    # no learning!
    def generateMLN(self, dbs, mln):
        newMLN = mln.duplicate()
        propsFound = {}
        for db in dbs:
            for oq in db.query('object(?cluster, ?objID)'):
                objID = oq['?objID']
            props = self.processDB(db)
            props.pop('cluster', 'cluster')
            if objID not in propsFound:
                propsFound[objID] = props
            else:
                tmp = propsFound[objID]
                for p in props:
                    if p not in tmp: 
                        tmp[p] = props[p]
                    else:
                        tmp[p].extend(props[p])
                        tmp[p] = list(set(tmp[p]))
                propsFound[objID] = tmp
        print propsFound
        maxLen = 1.
        for objID in propsFound:
            temp = propsFound[objID].copy()
            for p1 in propsFound[objID]:
                temp.pop(p1)
                for p2 in temp:
                    maxLen = max(len(propsFound[objID][p1]), len(temp[p2]))
                    for w1 in propsFound[objID][p1]:
                        for w2 in temp[p2]:
                            newMLN.addFormula('object(?cluster,{0}) ^ {1}(?cluster,{2}) ^ {3}(?cluster,{4})'.format(objID, p1, w1, p2, w2), weight=1./maxLen)
        return newMLN