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
import java
from praclog import logging
import jpype
from nlparsing import StanfordParser
import os
from prac.wordnet import WordNet, known_concepts
from mln.mln import readMLNFromFile
from mln.database import Database, readDBFromFile
from wnsenses import posMap
from itertools import chain
import nltk
from collections import defaultdict
import sys
from utils.graphml import to_html_hex
from prac.core import PRAC

# nltk.data.path = [os.path.join('..', '..', 'data', 'nltk_data')]
# wordnet_data_path = os.path.join('..', '..', 'data', 'wordnet')
# java.classpath.append(os.path.join('..', '..', '3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))
# grammar_path = grammarPath = os.path.join('..', '..', '3rdparty', 'stanford-parser-2012-02-03', 'grammar', 'englishPCFG.ser.gz')

def get_prob_color(p):
    '''
    Returns an HTML-hex string color value for the given probability
    between green (p=1.0) and red (p=0.0).
    ''' 
    return to_html_hex(.3, p * .75, 1., 'hsv')
#     return to_html_hex(p / 3., 1., 1., 'hsv')


def add_word_evidence_complete(db, word, pos, wn):
    '''
    Takes a word and its part of speech and adds fuzzy evidence
    over the _whole_ model (ie. all concepts db.mln). Its also sets
    the evidence for all concepts that are exluded by the possible
    word senses to false. If no part of speech is given, this (negative) evidence
    is not set, resulting in a-priori inference over senses.
    If a word is is not contained in wordnet (ie. has no applicable sense)
    then it is ignored.
    '''
    log = logging.getLogger()
    log.setLevel(logging.INFO)
    # collect a complete list of concepts
    pos = posMap.get(pos, None)
    if pos is not None:
        if len(wn.synsets(word, pos)) == 0: return
    all_concepts = map(lambda c: wn.synset(c), wn.known_concepts)
    log.info(all_concepts)
#     mln_concepts = db.mln.domains['concept']
#     for concept in mln_concepts:
#         synset = wn.synset(concept)
#         all_concepts.update(chain(*wn.hypernym_paths(synset)))
    # introduce a sense for each concept and add its similarity
    # to the concepts contained in the mln
    for synset in all_concepts:
        sense_id = synset.name#'%s-%.2d' % (word_const, i+1) # the synset name is the sense-id
        add_similarities(db, word, sense_id, synset, wn)
    if pos is None: return
    # a POS is given. Exclude all inapplicable senses and their hypernyms
    # as far as they are not a hypernym of an applicable sense
    # collect applicable hypernyms
    poss_senses = wn.synsets(word, pos)
    appl_hypernyms = set()
    for synset in poss_senses:
        appl_hypernyms.update(chain(*wn.hypernym_paths(synset)))
    # assert evidence
    for concept in all_concepts:
        if concept in appl_hypernyms: continue
        db.addGroundAtom('!has_sense(%s,%s)' % (word, concept.name))


def add_similarities(db, word, sense, concept, wn):
    mln_concepts = map(lambda c: wn.synset(c), db.mln.domains['concept'])
    for knwn_concept in mln_concepts:
        sim = wn.wup_similarity(concept, knwn_concept)
        db.addGroundAtom('is_a(%s,%s)' % (sense, knwn_concept.name), sim)
    
                
if __name__ == '__main__':
    
    prac = PRAC()
    prac.wordnet = WordNet(known_concepts)
    parser = prac.getModuleByName('nl_parsing')
    senses = prac.getModuleByName('wn_senses')
    wsd = prac.getModuleByName('wsd')
    
    logging.getLogger().setLevel(logging.DEBUG)
    gaussianPriorSigma=2
    learned_mln_path = os.path.join('/', 'home', 'nyga', 'work', 'code', 'prac', 'tmp', 'model-learned.mln') 
    
    learn = True
    learn = False
    
    # inference
    if not learn:
        sentence = 'fill the water into a bowl.'
        
        wsd.load_mln(learned_mln_path)
        g = prac.wordnet.asGraphML()
        for db in parser.parse(sentence):
#             db_ = senses.get_senses_and_similarities(db, wsd.get_known_concepts())
#             db_.printEvidence()
            evidence = db.union(wsd.mln, db)
#             senses_db = wsd.get_most_probable_senses(evidence)
            
            for q in db.query('has_pos(?w,?pos)'):
                w = q['?w'].split('-')[0]
                pos = q['?pos']
#                 if pos != 'NN': continue
                add_word_evidence_complete(evidence, w, pos, prac.wordnet)
            evidence.printEvidence()
            mrf = wsd.mln.groundMRF(evidence, method='FastConjunctionGrounding', useMultiCPU=True)
            result = mrf.inferEnumerationAsk(['has_sense'], None, shortOutput=True, useMultiCPU=True)
            colors = {}
#             senses_db.printEvidence()
            for word in mrf.domains['word']:
                max_prob = 0
                for q, v in result.iteritems():
                    pred, args = wsd.mln.logic.parseAtom(str(q))
                    if pred != 'has_sense': continue
                    w = args[0]
                    if w != word: continue
                    conceptname = args[1]
                    if v > max_prob: max_prob = v
                    color = v
        #             print v, q, color
                    colors[conceptname] = color
                for c, p in colors.iteritems():
                    colors[c] = get_prob_color(p / float(max_prob))
        #         print colors
                for n in g.nodes:
                    label = n.label
                    if colors.get(label, None) is not None:
                        n.color = colors[label]
                        print n.label, colors[label]
        #         print word
                graphpath = os.path.join('/', 'home', 'nyga', 'work', 'code', 'prac', 'tmp', '%s.graphml' % word)
                print ('writing %s' % graphpath)
                g.write(open(graphpath, 'w+'))
    else:    
        # learning
        mln = readMLNFromFile(os.path.join('/', 'home', 'nyga', 'work', 'code', 'prac', 'tmp', 'model.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
        dbs = readDBFromFile(mln, os.path.join('/', 'home', 'nyga', 'work', 'code', 'prac', 'tmp', 'training.db'))
        training_dbs = []
        for db, sim in zip(dbs, senses.get_similarities(*dbs)):
            db_ = db.union(None, sim)
#             db_ = db
            training_dbs.append(db_)
            db_.printEvidence()
            print '+++++++++'
        learned_mln = mln.learnWeights(training_dbs, method='BPLL_CG', gaussianPriorSigma=gaussianPriorSigma, useMultiCPU=True)
        learned_mln.writeToFile(learned_mln_path)
    
    
    
    exit(0)
    
    log = logging.getLogger()
    log.setLevel(logging.DEBUG)
    
    
    
    
    
    
#     wn_concepts = ['batter.n.02', 'milk.n.01', 'water.n.06', 'cup.n.01', 'cup.n.02', 'glass.n.02', 'bowl.n.01', 'bowl.n.03', 'coffee.n.01', 'bowl.n.04']
    
    if not java.isJvmRunning():
            java.startJvm()
    logging.getLogger().info('initializing nl_parsing')
    parser = StanfordParser(grammarPath)
    # this fixes some multithreading issues with jpype
    if not jpype.isThreadAttachedToJVM():
        jpype.attachThreadToJVM()

    # # # # # # # # # # # # # # # # # # # # # #
    # parse the sentence and create a database
    mln = readMLNFromFile(os.path.join('..', '..', 'pracmodules', 'nl_parsing', 'mln', 'nl_parsing.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
    db = Database(mln)
    deps = parser.getDependencies(sentence, True)
    deps = map(str, deps)
    words = set()
    for d in deps:
        db.addGroundAtom(d)
        f = mln.logic.parseFormula(str(d))
        words.update(f.params)
        log.debug(f)
    posTags = parser.getPOS()
    for pos in posTags.values():
        if not pos[0] in words:
            continue
        posTagAtom = 'has_pos(%s,%s)' % (pos[0], pos[1])
        db.addGroundAtom(posTagAtom)

    # # # # # # # # # # # # # # # # # # # # # #
    # create a WordNet taxonomy
    wn = WordNet()
    g = wn.asGraphML()
    inf_mln = readMLNFromFile(os.path.join('/', 'home', 'nyga', 'work', 'nl_corpora', 'wikihow', 'wts.pybpll_cg.Filling-new-1.mln'), logic='FuzzyLogic', grammar='PRACGrammar')
#     mln.declarePredicate('is_a', ['sense', 'concept'])
#     mln.declarePredicate('has_sense', ['sense', 'concept'], [False, True])
    print inf_mln.predicates
    print inf_mln.domains
#     mln.write(sys.stdout)
    ev_db = db.duplicate(inf_mln)
    
    # # # # # # # # # # # # # # # # # # # # # #
    # add possible word senses
    word2senses = defaultdict(list)
    word2hypernyms = defaultdict(list)
#     ev_db.addGroundAtom('action_role(w1-cond, Goal)')
#     ev_db.domains['word'] = ['w1']
#     ev_db.addGroundAtom('has_sense(w2, s1)')
#     add_similarities(ev_db, 'w2', 's1', wn.synset('fill.v.01'), wn)
# #     ev_db.addGroundAtom('action_role(w2-cond, Theme)')
# #     add_word_evidence_complete(ev_db, 'w1', None, wn)
#     add_word_evidence_complete(ev_db, 'w1', None, wn)
    
    for res in db.query('has_pos(?word,?pos)'):
        log.info('preparing query for %s' % str(res))
        word_const = res['?word']
        pos = posMap.get(res['?pos'], None)
        if pos is None:
            continue
        word = word_const.split('-')[0]
        add_word_evidence_complete(ev_db, word, pos, wn)


    ev_db.printEvidence()
    
    mrf = inf_mln.groundMRF(ev_db, method='FastConjunctionGrounding', useMultiCPU=True)
    log.info(mrf.evidence)
    result = mrf.inferEnumerationAsk(['has_sense'], None, shortOutput=True, useMultiCPU=True)
    colors = {}
    for word in mrf.domains['word']:
        max_prob = 0
        for q, v in result.iteritems():
            pred, args = mln.logic.parseAtom(str(q))
            if pred != 'has_sense': continue
            w = args[0]
            if w != word: continue
            conceptname = args[1]
            if v > max_prob: max_prob = v
            color = v
#             print v, q, color
            colors[conceptname] = color
        for c, p in colors.iteritems():
            colors[c] = get_prob_color(p / float(max_prob))
#         print colors
        for n in g.nodes:
            label = n.label
            if colors.get(label, None) is not None:
                n.color = colors[label]
                print n.label, colors[label]
#         print word
        g.write(open('%s.graphml' % word, 'w+'))
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    