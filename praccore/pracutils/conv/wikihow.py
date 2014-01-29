# 
#
# (C) 2011-2014 by Daniel Nyga (nyga@cs.uni-bremen.de)
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
from praclog import logging
from nltk.corpus import wordnet as wn
from nlparsing import StanfordParser
import java
import itertools

PRAC_PATH = os.environ['PRAC_HOME']
java.classpath.append(os.path.join(PRAC_PATH, '3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))
grammarPath = os.path.join(PRAC_PATH, '3rdparty', 'stanford-parser-2012-02-03', 'grammar', 'englishPCFG.ser.gz')
# java.classpath.append(os.path.join(PRAC_PATH, '3rdparty', 'stanford-parser-2014-01-04', 'stanford-parser.jar'))
# grammarPath = os.path.join(PRAC_PATH, '3rdparty', 'stanford-parser-2014-01-04', 'grammar', 'englishPCFG.ser.gz')


def lexkey2synset(key):
    log = logging.getLogger(__name__)
    try:lemma, idx = key.split('%')
    except: log.error(key)
    for l in wn.lemmas(lemma):
        log.debug(l)
        if l.key.lower() == key.lower():
            return l.synset
    log.error('No synset found for %s' % key)
    

def readFromFile(filename):
    log = logging.getLogger(__name__)
    # load the parser
    parser = StanfordParser(grammarPath)
    
    f = open(filename)
    lines = f.readlines()
    sentence = []
    sentenceIdx = 0
    gndAtoms = []
    widx = 0
    # keep this list for a sanity check with the parser
    word_constants = set()
    parsed_constants = set()
    nullsense_constants = set()
    for lineIdx, l in enumerate(lines):
#         if sentenceIdx > 20:
#             break
        tokens = l.strip().split('\t')
        sidx = int(tokens[0])
#         widx = int(tokens[1])
        word = tokens[2]
        # replace spaces by underscores
        word = word.replace(' ', '_')
#         word = word.replace(r'\/', '/')
#         if word not in ('.', ',', '!', '?', ';'):
        if sentenceIdx < sidx:# or lineIdx == len(lines) - 1: # new stentence begins
            # parse the sentence here
            sentence = ' '.join(sentence).strip()
            print '//', sentence
            log.info('parsing sentence: "%s"' % sentence)
            deps = parser.getDependencies(sentence, True)
            log.info(deps)
            parsed_constants = set(itertools.chain(*map(lambda d: [str(d.dep().label()), str(d.gov().label())], deps)))
            log.info(parsed_constants)
            for d in map(str, deps):
                gndAtoms.append(d)
            parsed_constants = set([d for d in parsed_constants if d.find('ROOT') == -1])
            posTags = parser.getPOS()
            for pos in posTags.values():
                if pos[0] not in parsed_constants: continue
                gndAtoms.append('has_pos(%s,%s)' % (pos[0], pos[1]))
            for nullsense in nullsense_constants:
                if nullsense in parsed_constants:
                    gndAtoms.append('has_sense(%s, null)' % nullsense)
            if not sanity_check(word_constants, parsed_constants):
                log.error('constants are inconsistent: \n%s\n%s' % (sorted(word_constants), sorted(parsed_constants)))
                for d in deps:
                    print str(d)
                assert sanity_check(word_constants, parsed_constants)
            for atom in gndAtoms:
                print atom
            print '---'
            word_constants = set()
            sentence = []
            gndAtoms = []
            sentenceIdx = sidx
            widx = 0
            nullsense_constants = set()
#         print tokens
        if len(tokens) > 3:
            synset = tokens[3]
        else: synset = None
        
        widx += 1
        word_const = '%s-%d' % (word, widx)
        word_constants.add(word_const)
        sentence.append(word)

        if len(tokens) > 4:
            lexkey = tokens[4]
            synset = lexkey2synset(lexkey)
            sense_const = '%s-sense' % word_const
            gndAtoms.append('has_sense(%s,%s)' % (word_const, sense_const))
            gndAtoms.append('is_a(%s,%s)' % (sense_const, synset.name))
        else: 
            nullsense_constants.add(word_const)
        if len(tokens) > 5:
            role = tokens[5].strip()
            if role != '':
                gndAtoms.append('action_role(%s,%s)' % (word_const, role))
            
            
def sanity_check(constants, deps):
    '''
    Performs a simple sanity check of the parse by
    comparing the word constants fro the parser with the generated ones.
    '''
    atoms_gen = set(constants)
    atoms_parse = deps
    return atoms_parse.issubset(atoms_gen)
    
    
    

if __name__ == '__main__':
    if not java.isJvmRunning():
        java.startJvm()
    logging.getLogger().setLevel(logging.ERROR)
    readFromFile(os.path.join('/', 'home', 'nyga', 'work', 'nl_corpora', 'wikihow', 'stirring.roles'))
    
    java.shutdownJvm()