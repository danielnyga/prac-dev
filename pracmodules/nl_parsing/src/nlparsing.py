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

from prac.core import PRACModule, PRACPIPE
import jpype
import java
import re
import os
from mln.MarkovLogicNetwork import readMLNFromFile
from logic.grammar import parseFormula
import logging
import sys
from mln.database import Database
from actioncore.inference import PRACInferenceStep

java.classpath.append(os.path.join('3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))
grammarPath = os.path.join('3rdparty', 'stanford-parser-2012-02-03', 'grammar', 'englishPCFG.ser.gz')


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
        return '%s(%s, %s)' % (self.depName, self.dep, self.gov)

class StanfordParser(object):
    '''
    Python Wrapper for the Java implementation of the Stanford natural-language parser.
    '''
    
    def __init__(self, pcfg_model_fname=None):
        self.pcfg_model_fname = pcfg_model_fname
        self.package_lexparser = jpype.JPackage("edu.stanford.nlp.parser.lexparser")
        self.package_trees = jpype.JPackage('edu.stanford.nlp.trees')
        self.package = jpype.JPackage("edu.stanford.nlp")
        self.parser = self.package_lexparser.LexicalizedParser(self.pcfg_model_fname, ['-retainTmpSubcategories', '-maxLength', '160'])
        
    def getDependencies(self, sentence=None, collapsed=False):
        '''
        Returns the syntactic dependencies from the parse
        applied to the given sentence.
        ''' 
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
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
    Extracts syntactic features that are obtained by the parser, such as
    'has_pos' and the stanford dependencies.
    '''
    
    stanford_parser = None
    
#     def __init__(self, prac):
#         PRACModule.__init__(self, prac)
    
    def initialized(self):
        java.startJvm()
        self.prac.log.debug('initializing nl_parsing')
        if NLParsing.stanford_parser is None:
            NLParsing.stanford_parser = StanfordParser(grammarPath)
        # this fixes some multithreading issues with jpype
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()
        self.syntax_mln = readMLNFromFile(os.path.join(self.module_path, 'mln', 'nl_parsing.mln'))

    @PRACPIPE
    def run(self, pracinference):
        log = logging.getLogger('NLP')
        log.debug('Running %s' % self.name)
        parser = NLParsing.stanford_parser
        inferenceStep = PRACInferenceStep(pracinference, self)
        for instr in inferenceStep.pracinference.instructions:
            db = Database(self.syntax_mln)
            deps = parser.getDependencies(instr, True)
            deps = map(str, deps)
            words = set()
            for d in deps:
                db.addGroundAtom(d)
                f = parseFormula(str(d))
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
            for e, t in db.evidence.iteritems():
                log.debug((e, t))
            inferenceStep.output_dbs.append(db)
        return inferenceStep
            
            
            