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
import subprocess
import re
import os
import sys

import jpype

from prac.core.base import PRACModule, PRACPIPE, PRAC_HOME
from prac.core.inference import PRACInferenceStep
from prac import java
from pracmln import Database, MLN
from pracmln.mln.util import colorize
from pracmln.praclog import logger
from pracmln.utils.visualization import get_cond_prob_png


log_ = logger(__name__)
java.classpath.append(
    os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2012-02-03',
                 'stanford-parser.jar'))
grammarPath = os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2012-02-03',
                           'grammar', 'englishPCFG.ser.gz')


class ParserError(Exception):
    def __init__(self, *args, **margs):
        Exception.__init__(self, *args, **margs)


class Dependency(object):
    """
    This class represents one single Stanford dependency.
    """


    def __init__(self, depname, dep, gov):
        self.depName = depname
        self.dep = dep
        self.gov


    def __str__(self):
        return '{}({},{})'.format(self.depName, self.dep, self.gov)


class StanfordParser(object):
    """
    Python Wrapper for the Java implementation of the Stanford
    natural-language parser.
    """


    def __init__(self, pcfg_model_fname=None):
        self.pcfg_model_fname = pcfg_model_fname
        self.package_lexparser = jpype.JPackage(
            "edu.stanford.nlp.parser.lexparser")
        self.package_trees = jpype.JPackage('edu.stanford.nlp.trees')
        self.package = jpype.JPackage("edu.stanford.nlp")
        self.parser = self.package_lexparser.LexicalizedParser(
            self.pcfg_model_fname,
            ['-retainTmpSubcategories', '-maxLength', '160'])
        self.parse = None


    def get_dependencies(self, sentence=None, collapsed=False):
        """
        Returns the syntactic dependencies from the parse
        applied to the given sentence.
        """
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
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
        """
        Parses the sentence string, returning the tokens, and the parse tree
        as a tuple. tokens, tree = parser.parse(sentence)
        """
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
    """
    Extracts syntactic features that are obtained from the parser, such as
    'has_pos' and the stanford dependencies.
    """


    def __init__(self, prac):
        PRACModule.__init__(self, prac)
        self.mln = None


    def initialize(self):
        log_.info('initializing nl_parsing')

        self.mln = MLN(mlnfile=os.path.join(self.module_path, 'mln',
                                            'predicates.mln'),
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


    def extract_multiple_action_cores(self, db):
        dbs = []
        verb_list = self.get_all_verbs(db)
        if len(verb_list) < 2:
            return [db]

        # TODO improve the handling
        # Handle sentence with start with .....
        if len(verb_list) == 2:
            db.write()
            for word in ['start', 'Start']:
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
                    _, pred, args = db.mln.logic.parse_literal(atom)

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

                        if dependency_word and (
                                        dependency_word not in verb_list or
                                        pred == "event") and (
                                    dependency_word not in processed_word_set):
                            if pred != 'event' or not is_condition:
                                db_ << atom
                            if pred != 'has_pos' and pred != 'event':
                                remaining_word_set.add(dependency_word)
                processed_word_set.add(processed_word)
            dbs.append(db_)
        return dbs


    @PRACPIPE
    def __call__(self, pracinference):

        log_.info('Running {}'.format(self.name))
        step = PRACInferenceStep(pracinference, self)

        print colorize('+==========================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: PARSING NATURAL LANGUAGE |',
                       (None, 'green', True), True)
        print colorize('+==========================================+',
                       (None, 'green', True), True)
        print
        print colorize('Parsing NL instructions:', (None, 'white', True),
                       True), ' '.join(pracinference.instructions)

        cmd = "python {} '{}'".format(os.path.join(self.module_path, 'src',
                                                   'caller.py'),
                                      "' '".join(pracinference.instructions))

        print cmd
        res = subprocess.check_output(cmd, shell=True)

        # separate dbs
        dbs = res.split('---\n')
        if '' in dbs:
            dbs.remove('')

        # read db entries
        pngs = {}
        for i, db_ in enumerate(dbs):
            db = Database(self.mln)
            sp = db_.split('\n')
            if '' in sp:
                sp.remove('')

            for r in sp:
                db << r

            step.output_dbs.extend(
                self.extract_multiple_action_cores(db))

            print
            print colorize('Syntactic evidence:', (None, 'white', True),
                           True)
            db.write(sys.stdout, True)

            print ','.join(pracinference.instructions)

            pngs['NL Parsing - ' + str(i)] = get_cond_prob_png(
                ','.join([x.name for x in self.mln.predicates[:10]]) + ',...',
                str(','.join(pracinference.instructions)), filename=self.name)
            step.png = pngs
        return step
