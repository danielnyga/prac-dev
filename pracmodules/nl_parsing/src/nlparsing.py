# PROBABILISTIC ROBOT ACTION CORES 
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

from prac.core import PRACModule, PRACPIPE, PRAC_HOME
from prac.inference import PRACInferenceStep
import jpype
import java
import re
import os
from mln import readMLNFromFile
import logging
from mln.database import Database
from utils import colorize
import sys

os.environ['JAVA_HOME'] = '/opt/Oracle_Java/jdk1.8.0_66/'
os.environ['NLP_PARSER'] = '/home/seba/workspace/prac/3rdparty/stanford-parser-2015/edu/stanford/nlp/models/lexparser/englishPCFG.ser.gz'
java.classpath.append(os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2015', 'stanford-parser.jar'))
grammarPath = os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2015', 'grammar', 'englishPCFG.ser.gz')


class ParserError(Exception):
    def __init__(self, *args, **margs):
        Exception.__init__(self, *args, **margs)
        
class Dependency(object):
    '''
    This class represents one single Stanford dependency.
    '''
        
    def __init__(self, depName, dep, gov):
        self.depName = depName
        self.dep = dep
        self.gov
        
    def __str__(self):
        return '%s(%s,%s)' % (self.depName, self.dep, self.gov)

class StanfordParser(object):
    '''
    Python Wrapper for the Java implementation of the Stanford natural-language parser.
    '''
    
    def __init__(self, pcfg_model_fname=None):
        self.pcfg_model_fname = pcfg_model_fname
        self.package_lexparser = jpype.JPackage("edu.stanford.nlp.parser.lexparser")
        self.package_trees = jpype.JPackage('edu.stanford.nlp.trees')
        self.package = jpype.JPackage("edu.stanford.nlp")
        #self.parser = self.package_lexparser.LexicalizedParser(self.pcfg_model_fname, ['-retainTmpSubcategories', '-maxLength', '160'])
        self.parser = jpype.JClass("edu.stanford.nlp.parser.lexparser.LexicalizedParser").loadModel()
        
    def getDependencies(self, sentence=None, collapsed=False):
        '''
        Returns the syntactic dependencies from the parse
        applied to the given sentence.
        ''' 
        if sentence is not None:
            self.parse = self.parser.parse(sentence)
        tlp = self.package_trees.PennTreebankLanguagePack()
        puncWordFilter = tlp.punctuationWordRejectFilter()
        gsf = tlp.grammaticalStructureFactory(puncWordFilter)
        gs = gsf.newGrammaticalStructure(self.parse)
        if collapsed is True:
            deps =  gs.typedDependenciesCollapsed()
        else:
            deps = gs.typedDependencies()
        return deps
    
    def getPOS(self, sentence=None):
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
        tokens = [str(x) for x in self.parse.taggedYield()]
        pos = dict()
        commaOffset = 0
        for i,t in enumerate(tokens):
            w,p = t.rsplit('/',1)
            if not re.match(r'[a-zA-z0-9]+', p): 
                continue
            pos[i+1]=(['%s-%d'% (w, i+1-commaOffset), p])
        return pos
        

    def printInfo(self):
        Numberer = self.package.util.Numberer
        print ("Grammar\t" + 
               `Numberer.getGlobalNumberer("states").total()` + '\t' + 
               `Numberer.getGlobalNumberer("tags").total()` + '\t' + 
               `Numberer.getGlobalNumberer("words").total()` + '\t' + 
               `self.parser.pparser.ug.numRules()` + '\t' + 
               `self.parser.pparser.bg.numRules()` + '\t' + 
               `self.parser.pparser.lex.numRules()`)

        print "ParserPack is ", self.parser.op.tlpParams.getClass()
        print "Lexicon is ", self.parser.pd.lex.getClass()        
        print "Tags are: ", Numberer.getGlobalNumberer("tags")
        self.parser.op.display()
        print "Test parameters"
        self.parser.op.tlpParams.display();
        self.package_lexparser.Test.display()
        
        
    def parse(self, sentence):
        '''
        Parses the sentence string, returning the tokens, and the parse tree as a tuple.
        tokens, tree = parser.parse(sentence)
        '''
        tokens = self.documentPreprocessor.getWordsFromString(sentence)
        for token in tokens:
            if token.word() in ["down"]:
                print "setting tag"
                token.setTag("IN")
                pass
            if token.word().lower() in ["bot"]:
                token.setTag("NN")
                pass
        
        wasParsed = self.parser.parse(tokens)
        if not wasParsed:
            raise ParserError("Could not parse " + sentence)
        return tokens, self.parser.getBestParse()


class NLParsing(PRACModule):
    '''
    Extracts syntactic features that are obtained from the parser, such as
    'has_pos' and the stanford dependencies.
    '''
    
    def __init__(self, prac):
        PRACModule.__init__(self, prac)
        self.stanford_parser = None
        if not java.isJvmRunning():
            java.startJvm()
    
    def initialize(self):
        logging.getLogger().info('initializing nl_parsing')
        # this fixes some multithreading issues with jpype
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()
        if self.stanford_parser is None:
            self.stanford_parser = StanfordParser(grammarPath)
        self.mln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'predicates.mln'), grammar='PRACGrammar', logic='FuzzyLogic')

    
    def parse_without_prac(self, *sentences):
        '''
        Returns a Database or a list of Databases with the syntactic dependencies
        and part-of-speech tags from the natural-language parser.
        '''
        log = logging.getLogger(self.__class__.__name__)
        parser = self.stanford_parser
        for s in sentences:
            db = Database(self.mln)
            deps = parser.getDependencies(s, True)
            deps = map(str, deps)
            words = set()
            for d in deps:
                db.addGroundAtom(d)
                f = self.mln.logic.parseFormula(str(d))
                words.update(f.params)
                log.info(f)
            posTags = self.stanford_parser.getPOS()
            pos = []
            for pos in posTags.values():
                if not pos[0] in words:
                    continue
                posTagAtom = 'has_pos(%s,%s)' % (pos[0], pos[1])
                pos.append(posTagAtom)
                db.addGroundAtom(posTagAtom)
                posTags[pos[0]] = pos[1]
            yield db
    

    def shutdown(self):
        if java.isJvmRunning():
            java.shutdownJvm()

    @PRACPIPE
    def __call__(self, pracinference):
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()
        log = logging.getLogger()
        log.info('Running %s' % self.name)
        parser = self.stanford_parser
        inferenceStep = PRACInferenceStep(pracinference, self)
        
        print colorize('+==========================================+', (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: PARSING NATURAL LANGUAGE |', (None, 'green', True), True)
        print colorize('+==========================================+', (None, 'green', True), True)
        print 
        for instr in pracinference.instructions:
            print colorize('Parsing NL instruction:', (None, 'white', True), True), instr
        
            db = Database(self.mln)
            deps = parser.getDependencies(instr, True)
            deps = map(str, deps)
            words = set()
            for d in deps:
                if d.startswith('nmod:agent'):
                    d = d.replace('nmod:',"",1)
                elif d.startswith('nmod:'):
                    d = d.replace('nmod:',"prep_",1)
                db.addGroundAtom(d)
                f = self.mln.logic.parseFormula(str(d))
                words.update(f.params)
                log.debug(f)
            self.posTags = self.stanford_parser.getPOS()
            self.pos = []
            for pos in self.posTags.values():
                if not pos[0] in words:
                    continue
                posTagAtom = 'has_pos(%s,%s)' % (pos[0], pos[1])
                self.pos.append(posTagAtom)
                db.addGroundAtom(posTagAtom)
                self.posTags[pos[0]] = pos[1]
            inferenceStep.output_dbs.append(db)
            
            print
            print colorize('Syntactic evidence:', (None, 'white', True), True)
            db.write(sys.stdout, True)
            
            
        return inferenceStep

            