# PRAC -- SEMCORE CORPUS CONVERSION
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
import lxml.html.soupparser as parser
from mln.mln import Predicate
from praclog import logging
import os
import sys
from prac.core import PRAC
from prac.wordnet import WordNet
from nltk.corpus import wordnet
import java
import mln.database
from mln.errors import NoSuchPredicateError


java.classpath.append(os.path.join('3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))
grammarPath = os.path.join('3rdparty', 'stanford-parser-2012-02-03', 'grammar', 'englishPCFG.ser.gz')
prac = PRAC()
prac.wordnet = WordNet(concepts=None)

def readSemcor3File(filename):
    '''
    Reads an XML semcore3.0 file and returns a corresponding MLN database.
    '''
    print filename
    if not java.isJvmRunning():
            java.startJvm()
    log = logging.getLogger(__name__)
    tree = parser.parse(filename)
    
    parserSt = prac.getModuleByName('nl_parsing')
    parserSt.initialize()

    
    target = open(filename+".db", 'w')
    is_file_causing_parsing_errors = False
    for e in tree.iter():
        if e.tag == 's':
            s, atoms = reconstruct(e)
            target.write('//'+ s+"\n")
            for a in atoms:
                target.write(a+"\n")
            
            is_sentence_parsed = False
            
            while not is_sentence_parsed:
                try:
                    for x in parserSt.parse_without_prac(s):
                        x.write(target)
                        target.write('---\n')
                    is_sentence_parsed = True
                    print "Parsing successfully."
                except NoSuchPredicateError:
                    _, exc_value , _ = sys.exc_info()
                    print exc_value
                    predicate_name = str(exc_value).split(':')[1].strip()
                    parserSt.mln.declarePredicate(Predicate(predicate_name,['word','word']))
                    
                    print "Try it again."
                except:
                    is_sentence_parsed = True
                    is_file_causing_parsing_errors = True
                    print sys.exc_info()[0]
                
    target.close()
    if not is_file_causing_parsing_errors:
        os.remove(filename)

def reconstruct(s_element):
    sentence = []
    wf_count = 0
    gnd_atoms = []
    for e in s_element.iter():
        if e.text is not None:
            sentence.append(e.text)
	
        if e.tag == 'wf':
            wf_count += 1
            word_const = '%s-%d' % (e.text, wf_count)
            if e.get('pos', None) is not None:
                gnd_atoms.append('1.00 has_pos(%s,%s)' % (word_const, e.get('pos')))
            if e.get('lemma', None) is not None:
                lem = e.get('lemma')
                lexsn = e.get('lexsn')
                synset = None
                for l in wordnet.lemmas(lem):
                    if l.key == '%s%%%s' % (lem, lexsn):
                        synset = l.synset
                if synset is not None:
                    gnd_atoms.append('1.00 has_sense(%s,%s)' % (word_const, synset.name))
                    gnd_atoms.append('1.00 is_a(%s,%s)' % (synset.name, synset.name))
        
        elif e.tag == 'punc':
            wf_count += 1

    sentence = ' '.join(sentence).strip()
    return sentence, gnd_atoms
    
if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    path = os.path.join('/', 'home', 'seba', 'Desktop', 'semcor3.0', 'brown1', 'tagfiles')
    path2 = os.path.join('/', 'home', 'seba', 'Desktop', 'semcor3.0', 'brown2', 'tagfiles')
    path3 = os.path.join('/', 'home', 'seba', 'Desktop', 'semcor3.0', 'brownv', 'tagfiles')
    for filename in os.listdir(path):
        readSemcor3File(os.path.join(path, filename))
    #for filename in os.listdir(path2):
    #    readSemcor3File(os.path.join(path2, filename))
    #for filename in os.listdir(path3):
    #    readSemcor3File(os.path.join(path3, filename))
