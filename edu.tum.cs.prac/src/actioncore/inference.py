# PROBABILISTIC ROBOT ACTION CORES - INFERENCE
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

from actioncore.features import FeatureManager, Syntax, WordSenses, MissingRoles
from pracmln.PRACDatabase import PRACDatabase
from wcsp.converter import WCSPConverter
from utils import StopWatch, red, bold
from utils import bash
from grammar import parsePracFormula
from actioncore import PRAC, PRACReasoner, PRACPIPE
from nltk.corpus import wordnet as wn
from StanfordParser import Parser
#from linguistics.verbalizer import PRACVerbalizer
#from linguistics import NLISentence
import os
import re
import math
import operator

class PRACInit(PRACReasoner):
    
    def __init__(self, actioncore):
        super(PRACInit, self).__init__('init')
        self.prac = PRAC()
        self.actioncore = self.prac.action_cores[actioncore]
    
    @PRACPIPE
    def run(self):
        watch = self.pracinference.watch
        actioncore = self.actioncore
        self.pracinference.actioncore = actioncore
        mln = actioncore.learnedMLN.duplicate()
        self.pracinference.mlns['pracinit'] = mln
        
        # generate the feature extractors 
        features = self.pracinference.features

        synParser = Syntax(features)
        features.add(synParser)
        features.add(WordSenses(features))
        
        # feed evidence into the database
        db = PRACDatabase(mln)
        self.pracinference.databases['core'] = db
        feat = []
        feat.append(features.get('syntax'))
        feat.append(features.get('wordsenses'))
        for f in feat:
#            print "PRINTING F" + f
            f.run(self.pracinference)

#        print 
#        print bash.BOLD + 'Evidence' + bash.END
#        for e in sorted(db.evidence):
#            print e
#        
        queryPreds = ['has_sense', 'action_role']
        for pred in mln.predicates:
            if not pred in queryPreds:
                mln.closedWorldPreds.append(pred)

        # ground the MRF        
        watch.tag('MRF Construction')
        mrf = mln.groundMRF(db, simplify=True, method='WCSPGroundingFactory')
        
        # convert to WCSP
        watch.tag('WCSP Construction & Inference')
        conv = WCSPConverter(mrf)

        resultDB = PRACDatabase(mln, db=conv.getMostProbableWorldDB())
        self.pracinference.databases['core'] = resultDB
        
        
        print 
        print bash.BOLD + 'Syntactic Parser Features:' + bash.END
        print 
        for f in synParser.deps:
            print f
        
        print
        print bash.BOLD + 'Word Sense Disambiguation' + bash.END
        print bash.BOLD + '=========================' + bash.END
        for s in resultDB.query('has_sense(?w, ?s) ^ !is_a(?s, NULL)'):
            print bash.BOLD + s['?w'] + bash.END
            feature = features.get('wordsenses')
            senses = feature.words2senses[s['?w']]
            tick = senses.index(s['?s'])
            printWordSenses(map(lambda x: feature.senses2concepts[x], senses), tick)
            print
        print
        print
        print bash.BOLD + 'Action Role Assignment' + bash.END
        print bash.BOLD + '======================' + bash.END
        print
        print bash.BOLD + 'Action Core:' + bash.END + ' ' + actioncore.name
        print bash.BOLD + 'Roles:' + bash.END
        printRoles(resultDB, actioncore)
        print
        watch.finish()
        watch.printSteps()
        print
	       
        
    @PRACPIPE    
    def __call__(self, *args):
        self.pracinference = PRACInference(self.prac)
        self.pracinference.sentence = args[0]
        self.run()
        
    def getWordSenses(self, world):
        """Return the word senses as a dictionary."""

        result = dict()
        feature = self.features.get('wordsenses')        
        for i, word in enumerate(self.database.domains['word']):
            senseAtoms = feature.senseAtoms.get(word, None)
            result[word] = dict()
            if senseAtoms is None:
                result[word]['sense'] = 'NULL'
                continue
            for senseAtom in senseAtoms:
                tick = world[self.mrf.gndAtoms[senseAtom].idx]
                if tick: result[word]['sense'] = re.findall(r"[^(),]+",
                        senseAtom)[2]
        return result
    
    

class ActionCores(PRACReasoner):
    
    @PRACPIPE
    def run(self):
        pass

class ActionRoles(PRACReasoner):
    
    @PRACPIPE
    def run(self):
        infer = self.pracinference
        features = infer.features
        features.add(MissingRoles(features))
        
        watch = infer.watch
        prac = infer.prac
        watch.reset()
        watch.tag('Inferring missing roles')
        actioncore = infer.actioncore
        
        mln = actioncore.learnedMLN.duplicate()
        db = PRACDatabase(mln)
        infer.mlns['missingroles'] = mln
        infer.databases['missingroles'] = db
        
        feature = features.get('missingroles')
        feature.run(infer)
        
        print 
        
        print bash.BOLD + 'Instruction Completion' + bash.END
        print bash.BOLD + '======================' + bash.END

        print
        print bash.BOLD + 'Missing Roles:' + bash.END
        for role in feature.missingRoles:
            print '    ' + bash.BOLD + bash.RED + role + bash.END, '--', actioncore.action_roles[role].definition
        print
        if len(feature.missingRoles) == 0:
            print '    none'
        queryPreds = ['has_sense', 'has_pos']
        for pred in mln.predicates:
            if not pred in queryPreds:
                mln.closedWorldPreds.append(pred)
        
        mrf = mln.groundMRF(db, method='WCSPGroundingFactory')
        converter = WCSPConverter(mrf)
        
#        resultDB = PRACDatabase(mln, db=converter.getMostProbableWorldDB())
#        infer.databases['missingroles'] = resultDB
        
        self.pracinference.role_distribution = {}
        for role in feature.missingRoles:
            for concept in actioncore.known_concepts: break
            self.pracinference.role_distribution[role] = converter.getPseudoDistributionForGndAtom('has_sense(%s,%s)' % (role, concept))
            sortedList = sorted([(str(l.params[1]),v) for l, v in self.pracinference.role_distribution[role].iteritems()], key=operator.itemgetter(1), reverse=True)
            printDistribution(sortedList)
            infer.databases['missingroles'].addGroundAtom('action_role(%s,%s)' % (role, role))
            infer.databases['missingroles'].addGroundAtom('has_sense(%s,%s)' % (role, role))
            infer.databases['missingroles'].addGroundAtom('is_a(%s,%s)' % (role, sortedList[0][0]))
            print 
            print 'Most likely %s: ' % (role) + bash.OKGREEN + bash.BOLD + sortedList[0][0] + bash.END
            print
            print
            
                
        
#        for atom, prob in dist.iteritems():
#            print str(atom) + '\t' + str(prob)
        print
        print
        print bold('Instruction completed.')
        print
        print 
        
class PRACResult(PRACReasoner):
    
    def __init__(self):
        super(PRACResult, self).__init__('result')
    
    @PRACPIPE
    def run(self):
        pass

actioncores = ActionCores('actioncores')
actionroles = ActionRoles('actionroles')

    
class PRACInference(object):
    
    def __init__(self, prac):
        self.prac = prac
        self.features = FeatureManager(prac)
        self.possibleWorlds = []
        self.sentence = None
        self.watch = StopWatch()
        self.databases = {}
        self.mlns = {}
        
        grammarPath = os.path.join('3rdparty', 'stanford-parser-2012-02-03', 'grammar', 'englishPCFG.ser.gz')
        self.synParser = Parser(grammarPath)
 



def printWordSenses(senses, tickIdx):
    '''
    Prints the list of word senses and ticks the one specified by the given index.
    '''
    for idx, sense in enumerate(senses):
        if isinstance(sense, basestring):
            sense = wn.synset(sense)
        print '    [%s] %s: %s (%s)' % ('X' if tickIdx==idx else ' ', bash.BOLD + sense.name + bash.END, sense.definition, ';'.join(sense.examples))  

def printRoles(db, actioncore):
    for var in db.query('action_role(?w, ?r) ^ !(?r=NULL) ^ !(?r=ActionVerb)'):
        print '    ' + bash.OKGREEN + bash.BOLD + var['?r'] + bash.END + ': ' + var['?w'] + ' (%s)' %  actioncore.action_roles[var['?r']].definition
    

def printDistribution(distribution):
    maxlen = max(map(len, map(lambda x: str(x[0]), distribution)))
    for l, v in distribution:
        scale = 100
        barlen = int(math.floor(v * scale))
        l = str(l)
        print l.ljust(maxlen+1) + '[' + bash.OKGREEN + ('|' * barlen).ljust(scale) + bash.END + ']'
