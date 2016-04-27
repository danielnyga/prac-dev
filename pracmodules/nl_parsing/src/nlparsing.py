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

from prac.core.base import PRACModule, PRACPIPE, PRAC_HOME
from prac.core.inference import PRACInferenceStep
import jpype
import re
from prac import java
import os
import logging
import sys
from pracmln import Database, MLN
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize

java.classpath.append(os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))
grammarPath = os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2012-02-03', 'grammar', 'englishPCFG.ser.gz')


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
    Extracts syntactic features that are obtained from the parser, such as
    'has_pos' and the stanford dependencies.
    '''
    
    def __init__(self, prac):
        PRACModule.__init__(self, prac)
        self.stanford_parser = None
        if not java.isJvmRunning():
            java.startJvm()
            
    def is_aux_verb(self,word,db):
        #regex_aux = re.compile('aux\s*\(\s*(\w+)-{0,1}\d*\s*,\s*'+word+'\s*\)')
        #regex_auxpass = re.compile('auxpass\s*\(\s*\w+-{0,1}\d*\s*,\s*'+word+'\s*\)')
        
        for _ in db.query('aux(?w, {})'.format(word)):
            return True;
        
        for _ in db.query('auxpass(?w, {})'.format(word)):
            return True;
        
        return False;
    
    def get_all_verbs(self,db):
        valid_predicate_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
        verb_list=[]
        
        for q in db.query('has_pos(?w,?p)'):
            word_pos = q['?p']
            word = q['?w']
            
            if word_pos in valid_predicate_tags and not self.is_aux_verb(word, db):
                verb_list.append(word)
        
        #Sort list by the order occurrence of the verbs
        return sorted(verb_list, key=lambda verb: int(verb.split('-')[-1]))
      
    def extract_multiple_action_cores(self,db):
        dbs = []
        verb_list = self.get_all_verbs(db) 
        if len(verb_list) < 2:
            return [db]
        
        #TODO improve the handling
        #Handle sentence with start with .....
        if len(verb_list) == 2:
            db.write()
            for word in ['start','Start']:
                for _ in db.query('prepc_with({}-1,?p)'.format(word)):
                    return [db]
                
        for verb in verb_list:
            db_ = Database(self.mln)
            processed_word_set = set()
            remaining_word_set = set()
            remaining_word_set.add(verb)
            
            while remaining_word_set:
                processed_word = remaining_word_set.pop()
                is_condition = False
                
                for atom, _ in sorted(db.evidence.iteritems()):
                    _ , pred , args = db.mln.logic.parse_literal(atom)
                    
                    if len(args) == 1 and args[0] == processed_word: 
                        db_ << atom
                        is_condition = True
                    elif len(args) > 1:
                        word1 = args[0]
                        word2 = args[1]
                        
                        dependency_word = ""
                        if word1 == processed_word: 
                            dependency_word = word2
                        elif word2 == processed_word:
                            dependency_word = word1
                        
                        if dependency_word and (not dependency_word in verb_list or pred == "event") and (not dependency_word in processed_word_set):
                            if pred != 'event' or not is_condition: 
                                db_ << atom
                            if pred != 'has_pos' and pred != 'event':
                                remaining_word_set.add(dependency_word)
                processed_word_set.add(processed_word)
            dbs.append(db_)
        return dbs
    
    def initialize(self):
        logging.getLogger().info('initializing nl_parsing')
        # this fixes some multithreading issues with jpype
        if not jpype.isThreadAttachedToJVM():
            jpype.attachThreadToJVM()
        if self.stanford_parser is None:
            self.stanford_parser = StanfordParser(grammarPath)
        self.mln = MLN(mlnfile=os.path.join(self.module_path, 'mln', 'predicates.mln'), grammar='PRACGrammar', logic='FuzzyLogic')

    
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
                db << d
                f = self.mln.logic.parse_formula(str(d))
                words.update(f.args)
                log.debug(f)
            self.posTags = self.stanford_parser.getPOS()
            self.pos = []
            for pos in self.posTags.values():
                if not pos[0] in words:
                    continue
                posTagAtom = 'has_pos(%s,%s)' % (pos[0], pos[1])
                self.pos.append(posTagAtom)
                db << posTagAtom
                self.posTags[pos[0]] = pos[1]
            
            inferenceStep.output_dbs.extend(self.extract_multiple_action_cores(db))
            
            print
            print colorize('Syntactic evidence:', (None, 'white', True), True)
            db.write(sys.stdout, True)
            
            
        return inferenceStep
