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
import itertools
import json
import os
import re
import string
import subprocess
import sys
import jpype
from tempfile import NamedTemporaryFile

from prac_nltk import word_tokenize
from prac_nltk.corpus import wordnet as wn

from prac.core.base import PRACModule, PRACPIPE
from prac.core.errors import ParserError
from prac.core.inference import PRACInferenceStep
from prac.core.wordnet import WordNet
from prac.pracutils.utils import prac_heading
from pracmln import MLN
from pracmln.mln.database import parse_db
from pracmln.mln.util import colorize
from pracmln import praclog
from pracmln.utils.visualization import get_cond_prob_png


logger = praclog.logger(__name__, praclog.INFO)
wordnet = WordNet(concepts=None)


class StanfordParser(object):
    '''
    Python Wrapper for the Java implementation of the Stanford
    natural-language parser.
    '''


    def __init__(self, pcfg_model_fname=None):
        self.pcfg_model_fname = pcfg_model_fname
        self.package_lexparser = jpype.JPackage("edu.stanford.nlp.parser.lexparser")
        self.package_trees = jpype.JPackage('edu.stanford.nlp.trees')
        self.package = jpype.JPackage("edu.stanford.nlp")
        self.parser = self.package_lexparser.LexicalizedParser.loadModel(self.pcfg_model_fname, ['-retainTmpSubcategories', '-maxLength', '160'])
        self.parse = None


    def get_dependencies(self, sentence=None, collapsed=False):
        '''
        Returns the syntactic dependencies from the parse
        applied to the given sentence.
        '''
        if sentence is not None:
            self.parse = self.parser.parse(sentence)
        tlp = self.package_trees.PennTreebankLanguagePack()
        puncwordfilter = tlp.punctuationWordRejectFilter()
        gsf = tlp.grammaticalStructureFactory(puncwordfilter)
        gs = gsf.newGrammaticalStructure(self.parse)
        if collapsed is True:
            deps = gs.typedDependenciesCollapsed()
        else:
            deps = gs.typedDependencies()
        return deps


    def get_pos(self, sentence=None):
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
        tokens = [str(x) for x in self.parse.taggedYield()]
        pos = dict()
        commaoffset = 0
        for i, t in enumerate(tokens):
            w, p = t.rsplit('/', 1)
            if not re.match(r'[a-zA-z0-9]+', p):
                continue
            pos[i + 1] = (['%s-%d' % (w, i + 1 - commaoffset), p])
        return pos


    def print_info(self):
        numberer = self.package.util.Numberer
        print ("Grammar\t" +
               repr(numberer.getGlobalNumberer("states").total()) + '\t' +
               repr(numberer.getGlobalNumberer("tags").total()) + '\t' +
               repr(numberer.getGlobalNumberer("words").total()) + '\t' +
               repr(self.parser.pparser.ug.numRules()) + '\t' +
               repr(self.parser.pparser.bg.numRules()) + '\t' +
               repr(self.parser.pparser.lex.numRules()))

        print "ParserPack is ", self.parser.op.tlpParams.getClass()
        print "Lexicon is ", self.parser.pd.lex.getClass()
        print "Tags are: ", numberer.getGlobalNumberer("tags")
        self.parser.op.display()
        print "Test parameters"
        self.parser.op.tlpParams.display()
        self.package_lexparser.Test.display()


    def parse(self, sentence):
        '''
        Parses the sentence string, returning the tokens, and the parse tree
        as a tuple. tokens, tree = parser.parse(sentence)
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

        wasparsed = self.parser.parse(tokens)
        if not wasparsed:
            raise ParserError("Could not parse " + sentence)
        return tokens, self.parser.getBestParse()


class NLParsing(PRACModule):
    '''
    Extracts syntactic features that are obtained from the parser, such as
    'has_pos' and the stanford dependencies.
    '''


    def __init__(self, prac):
        PRACModule.__init__(self, prac)
        self.mln = None


    def initialize(self):
        logger.debug('initializing nl_parsing')

        self.mln = MLN(mlnfile=os.path.join(self.module_path, 'mln', 'predicates.mln'),
                       grammar='PRACGrammar', logic='FuzzyLogic')


    @staticmethod
    def is_aux_verb(word, db):

        for _ in db.query('aux(?w, {})'.format(word)):
            return True

        for _ in db.query('auxpass(?w, {})'.format(word)):
            return True

        return False


    def get_all_verbs(self, db):
        valid_predicate_tags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP']
        verb_list = []

        for q in db.query('has_pos(?w,?p)'):
            word_pos = q['?p']
            word = q['?w']

            if word_pos in valid_predicate_tags and not self.is_aux_verb(word,
                                                                         db):
                verb_list.append(word)

        # Sort list by the order occurrence of the verbs
        return sorted(verb_list, key=lambda verb: int(verb.split('-')[-1]))


    @staticmethod
    def find_compounds(sentence):
        '''
        Simple check for compound words in the input sentence (forward-check
        if 3-word or 2-word pairs exist in ontology) No check for POS tag
        included.
        Examples:
            * 'unload the washing machine and switch off the hair dryer'
                will be replaced by 'unload the washing_machine and switch
                off the hair_dryer'
            * 'start the external combustion engine' will be replaced by
                'start the external-combustion_engine'
        '''
        instr = word_tokenize(sentence)
        newinstr = []

        i = 0
        for _ in instr:
            found = False
            if i < len(instr):
                stop = ''
                # check three-word compounds
                for x in itertools.product('_-', repeat=2):
                    tmpword = '{}{}{}{}{}'.format(instr[i], x[0], instr[min(len(instr)-1, i+1)],
                                                  x[1], instr[min(len(instr)-1, i+2)])
                    if len(wn.synsets(tmpword)) > 0:
                        newinstr.append(tmpword+stop)
                        found = True
                        i += 3
                        break
                # check two-word compounds
                if not found:
                    for y in ['_', '-']:
                        tmpword = '{}{}{}'.format(instr[i], y, instr[min(len(instr)-1, i+1)])
                        if len(wn.synsets(tmpword)) > 0:
                            newinstr.append(tmpword+stop)
                            found = True
                            i += 2
                            break
                # leave current word as it is
                if not found:
                    newinstr.append(instr[i])
                    i += 1
            else:
                i+=1
        # untokenize sentence before returning.
        return "".join([" "+i if not i.startswith("'") and i not in string.punctuation else i for i in newinstr]).strip()

    
    def parse_instructions(self, sentences):
        '''
        Accepts as arguments a sentence or a list of sentences. Returns the
        syntactic structure of the sentences in form of MLN databases
        containing the respective atoms.
        '''
        #=======================================================================
        # Create a temporary file in which nlparse will write its result
        #=======================================================================
        filepath = None
        with NamedTemporaryFile(suffix='.db', delete=False) as f:
            filepath = f.name
        cmd = ['python', os.path.join(self.module_path, 'src', 'nlparse.py'), '--out-file', filepath]
        cmd.extend([json.dumps(s) for s in sentences])

        logger.debug('Calling Stanford Parser: '.format(cmd))
        subprocess.call(cmd)
        with open(filepath, 'r') as f:
            c = f.read()
            return parse_db(self.mln, c)


    
    @PRACPIPE
    def __call__(self, pracinference):

        # ======================================================================
        # Initialization
        # ======================================================================

        logger.debug('inference on {}'.format(self.name))

        if self.prac.verbose > 0:
            print prac_heading('Parsing Natural Language')

        logger.debug('Running {}'.format(self.name))
        step = PRACInferenceStep(pracinference, self)

        # ======================================================================
        # Preprocessing
        # ======================================================================

        processed_instructions = []
        for instruction in pracinference.instructions:
            processed_instructions.append(
                self.find_compounds(instruction))

        pracinference.instructions = processed_instructions

        # ======================================================================
        # Parsing Instructions
        # ======================================================================

        if self.prac.verbose > 0:
            print colorize('Parsing instructions:', (None, 'white', True), True), ' '.join(pracinference.instructions)

        dbs =  self.parse_instructions(pracinference.instructions)
        pngs = {}
        
        for i, db in enumerate(dbs):
            step.output_dbs.append(db)

            if self.prac.verbose > 1:
                print
                print colorize('Syntactic evidence:', (None, 'white', True), True)
                db.write(sys.stdout, True)
                print

            pngs['NL Parsing - ' + str(i)] = get_cond_prob_png(','.join([x.name for x in self.mln.predicates[:10]]) + ',...',
                                                               str(','.join(pracinference.instructions)), filename=self.name)
            step.png = pngs
        return step
