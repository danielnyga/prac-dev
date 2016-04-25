#Sentence Manager manages all sentences that are annotated
import sys
from annotation.utilities import helper
import os
import utils
import re
import logging
import jellyfish
from prac_nltk.corpus import wordnet as wn
from parsing import Trees
import utilities as ut
import csv
from copy import deepcopy
from statistics import database
from statistics.database import Insertion, Initialization
from parsing.Trees import SyntacticElement
from knowrobinterface import RealWorld

class SentenceManager:
    
    """
    Manages multiple sentences(one text).
    
    Public methods:
    __init__ -- Public constructor
    
    Public fields:
    sentences -- Dictionary containing all sentences of the text.
    Key is the line number of the sentence in the text.
    """
    
    def __init__(self, parser, sentenceInfo=None, entityDictionary=None,
                 genderDict=None, kr = None, text_name = None):
        
        """
        Public constructor.
        
        Keyword arguments:
        sentenceInfo -- A Dictionary containing the information for one
                        sentence.
        entityDictionary -- Dictionary that is used to keep track of all
                            entities in one Text.
        """
        
        logging.info('New SentenceManager created')
        self.kr = kr
        #Dictionary used to keep track of all entities in multiple sentences
        self.entityDict = dict()
        if genderDict is None:
            self.genderDict = self._getGenderDict()
        else:
            self.genderDict = genderDict
        self.dependencies = set()
        self.sentences = dict()
        self.annotationList = []
        self.parser = parser
        self.insert = Insertion()
        self.text_name = text_name
        if not sentenceInfo is None:
            #Transform this into all the annotation specifics
            for s in sentenceInfo:
                #Create a dictionary with all the sentences of the 
                #text with the line number as ID
                logging.info('Adding new sentence to the list')
                #try:
                self.sentences[(str(s['id']))] = SentenceEntity(s, 
                                                                parser, 
                                                                self.entityDict, 
                                                                s['id'])
                #except Exception as e:
                #    logging.error('Could not add sentence with ID %s to Dictionary. Msg.: %s', str(s['id']), e)
                #    continue
                logging.info("""Successfully added sentence with ID %s to 
                                the dictionary""", str(s['id']))
            logging.info('Finished adding sentences to the dictionary')
        else:
            logging.warning('No sentence given!')
        
        #Return the text as MLN representation
    def getMLN(self,textID, createPhrases = True, realWorld=False):
        result = ""
        #build entity dep
        #Here grouding could be done
        result = self.getFeatures(textID, realWorld=realWorld)
        
            #result += self.getPhraseDistanceMLN(textID, ['NP'])
        
        #result += '\n\n //Distance between phrases(govenor distance)\n'
        #result += self.getGovenorDistanceMLN(textID)
        
        return result
        #Return the text as String. Sentences separated by newline
        
    def getFeatures(self,textID, phraseDistance=True, realWorld=False):
        """Create the MLN features that are the evidence for the MLN."""
        res = ''
        res += self.getDependenciesMLN(textID)
        res += self.getSentenceMLN(textID)
        res += self.getWordDistance(textID)
        #res += self.getWordSimilarityMLN(textID)
        #res += self.getPhraseSimilarityMLN(textID)
        #res += self.getFilteredPhraseDistanceMLN(textID, ['NP'])
        #res += self.getAppositiveMLN(textID)
        #res += self.getStrictHeadMatchingMLN(textID)
        #res += self.getWordInclusionMLN(textID)
        #res += self.getIWithinIMLN(textID)
        #res += self.getModifierCompabilityMLN(textID)
        #res += self.getGenderMLN(textID)
        #res += self.getKnowRobObjectsMLN()
        res += self.getGroundingMLN(knowrob=True)
        #res += self.getModifiersMLN()
        if realWorld:
            res += self.createRealWorldMLN()
        
        return res
    

    def create_nltk_concept_taxonomie_mln(self, nltk_concepts):
        """Return the MLN for the nltk taxonomie of nltk concepts."""
        
        res = ""
        for obj in nltk_concepts.iterkeys():
            obj_mln = utils.toMLNCompatibleFormat(obj)
            res += "isInstanceOf({obj},{cid})\n".format(obj=obj_mln, 
                                                        cid=obj_mln)
            
            for concept in nltk_concepts[obj]:
                path = wn.synset(concept).hypernym_paths()
                path_set = set()
                for p in path:
                    path_set.update(p)
                for hypernym in path_set:
                    hypernym_mln = utils.toMLNCompatibleFormat(hypernym.name)
                    res += "isaI({cid},{concept})\n".format(cid=obj_mln, 
                                                        concept=hypernym_mln)
        return res

    def createRealWorldMLN(self):
        """Creates the objects that exist in the world."""
        
        mapper = RealWorld.Mapping()
        res = ""
        real_world = mapper.get_nltk_concepts()
        
        res = self.create_nltk_concept_taxonomie_mln(real_world)
        rels = RealWorld.Relations()
        res += str(rels)
        return res
        
    
    def saveTextToSQLite(self, database_name):
        """
        Save progress to SQLite.
        """
        
        ini = Initialization()
        conn = ini.get_db(database_name)
        c = conn.cursor()
        text_id = self.insert.insert_or_update_text(self.text_name, cursor = c)
        for sentence in self.sentences.itervalues():
            sentence.saveSentenceToSQLite(text_id, c)
        for dep in self.dependencies:
            dep.saveToSQLite(text_id, cursor = c)
        ini.finish_transaction(conn)
    
    def getKnowRobObjectsMLN(self):
        """Get all the Objects from KnowRob."""
        knowrob_objects = dict()
        for x in self.kr.knowrobQuery("owl_individual_of(A,knowrob:'SpatialThing-Localized')"):
            knowrob_objects[x['A']] = dict()
            for y in self.kr.knowrobQuery("owl_has('{item}',P,O)".format(
                                                                item = x['A'])):
                knowrob_objects[x['A']][y['P']] = y['O'] 
            
        #object_chars
        res = ''
        
        return res
    
    def getModifiersMLN(self):
        res = '\n// Modifiers:\n\n'
        cat = ['JJ','JJR','JJS','RB','RBR','RBS']
        for sentence in self.sentences.itervalues():
            #all elements that match the cat POS
            mods = sentence.tree.syntacticElements(cat)
            #all elements that could habe any of the cat POS according to WN
            pelems = sentence.tree.possibleElements(cat)
            #set of all modifiers that need to be queried
            modifier = set([x.element.name for x in mods + pelems])
            #query knowrob for all object that have characteristics from the words given
        
        return res
    

    def getKnowrobTaxonomieMLN(self, objects):
        res = ''
        res += '\n // KnowRob Object Taxonomie\n'
        for obj in objects:
            object_classes = set([x['A'] for x in self.kr.knowrobQuery("owl_has({obj}, rdf:type, A)".format(obj=obj))])
            for oc in object_classes:
                isa = set([x['A'] for x in self.kr.knowrobQuery(
                            "owl_individual_of('{oclass}', A)".format(oclass=oc))])
                for o in isa:
                    res += "isaI({obj},{o})\n".format(
                        obj=
                        utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(obj)).strip("'"), 
                        o=utils.toMLNCompatibleFormat(
                            ut.helper.handleSpecialChars(o)))
        
        return res

    def getGroundingMLN(self, knowrob = False):
        res = '\n// Grounded elements in the text:\n\n'
        res += "isaI(NULL,NULL)\n"
        for sentence in self.sentences.itervalues():
            for np in sentence.tree.inOrderLeafs():
                if not np.element.groundIdentifier is None:
                    res += self.get_grounding_mln_for_word(np.element)
                else:
                    res += "isGrounded({phrase}, {obj})\n".format(
                                            phrase=np.element.mlnidentifier, 
                                            obj="NULL")
        return res
    
    def get_grounding_mln_for_word(self, element):
        """Create the MLN for one word and the object it is grounded on."""
        
        objects = set()
        mapper = RealWorld.Mapping()
        kr_concepts = mapper.query_knowrob_for_concepts_of_objects(
                            [element.groundIdentifier.strip("'")])
        nltk_concepts = mapper.get_nltk_mappings_for_kr_objects(kr_concepts)
        res = ""
        for key in nltk_concepts.keys():
            res += "isGrounded({phrase}, {object})\n".format(
                                        phrase = element.mlnidentifier, 
                                        object=utils.toMLNCompatibleFormat(key))
        
        res += self.create_nltk_concept_taxonomie_mln(nltk_concepts)
        return res         
    
    def getGenderMLN(self,textID):
        """Get the gender of NPs with the help of a static dictionary."""
        
        res = '\n\n// Gender\n'
        delta = 2
        for sentence in self.sentences.itervalues():
            for np in sentence.tree.inOrderNPs():
                if np.expression in self.genderDict:
                    if self.genderDict[np.expression] > delta:
                        res += '{gender}Gender({p1})\n'.format(
                                gender=self.genderDict[np.expression]['gender'], 
                                p1 = np.element.mlnidentifier) 
        return res
    
    def _getGenderDict(self):
        this_dir, this_filename = os.path.split(__file__)
        base_dir = os.path.join(this_dir, 'div')
        filename = os.path.join(base_dir, 'confidenceGender.csv')
        res = dict()
        
        with open(filename, 'r') as csvfile:
            csvreader = csv.reader(csvfile, delimiter='\t')
            for row in csvreader:
                res[row[0]] = dict()
                res[row[0]]['gender'] = row[1]
                res[row[0]]['confidence'] = row[2]
                
        return res
    
    def getModifierCompabilityMLN(self, textID):

            cat = ['amod', 'num', 'poss', 'prep']
            threashold = 0.8
            
            res = '\n// Incompatible modifiers\n'
            mods = self._getSpecificDependencies(cat)
            for idx, modSentence in mods.iteritems():
                for mod in modSentence:
                    mod1Synsets = wn.synsets(mod.second.word)
                    for idx2, mod2Sentence in mods.iteritems():
                        for mod2 in mod2Sentence:
                            if mod2.typ == mod.typ:
                                mod2Synsets = wn.synsets(mod2.second.word)
                                bestMatch = self._getBestSynsetMatch(
                                                                    mod1Synsets, 
                                                                    mod2Synsets)
                                if bestMatch['score'] > threashold and not bestMatch['score'] == 1:
                                    res += self._createModifierCompabilityMLN(mod.first, mod2.first)
            return res

    def _createModifierCompabilityMLN(self, mod1, mod2):
        res = ''
        #Get the tree representation of the words
        leaf1 = self._getLeafFromWord(mod1)
        leaf2 = self._getLeafFromWord(mod2)
        #Get the NPs of the Leafs
        leaf1NPs = self.findNPs(leaf1)
        leaf2NPs = self.findNPs(leaf2)
        
        for np1 in leaf1NPs:
            for np2 in leaf2NPs:
                res += 'inCompatibleModifier({phrase1},{phrase2})\n'.format(phrase1=np1.element.mlnidentifier, phrase2 = np2.element.mlnidentifier)
        return res
    
    def _getLeafFromWord(self, word):
        """Return the leaf from the parse tree given a word entity."""
       
        for leaf in self.sentences[str(word.sentence)].tree.inOrderLeafs():
            if word.word + '-' + str(word.index) == leaf.element.identifier:
                return leaf
        return None
        
    def _getBestSynsetMatch(self, set1, set2):
        best = dict()
        best['syn1'] = None
        best['syn2'] = None
        best['score'] = 0
        for syn1 in set1:
            for syn2 in set2:
                score = syn1.wup_similarity(syn2)
                if score > best['score']:
                    best['score'] = score
                    best['syn1'] = syn1
                    best['syn2'] = syn2
        return best
        
    def _getSpecificDependencies(self, category):
        """Get the dependencies that are given by the list."""
        
        mods = dict()
        for idx, sentence in self.sentences.iteritems():
            mods[idx] = []
            for dep in sentence.dependencies:
                if dep.typ in category:
                    mods[idx].append(dep)
        return mods
    
    def getIWithinIMLN(self, textID):
        """Return all the true I-Within-I relations."""
        
        res = '\n//I-Within-I\n\n'
        for sentence in self.sentences.itervalues():
            for phrase in sentence.getPhrases():
                parent_phrases = self.find_phrases_on_path_to_root(phrase)
                for pnp in parent_phrases:
                    res += 'iWithini({phrase1},{phrase2})\n'.format(
                                        phrase1 = phrase.element.mlnidentifier, 
                                        phrase2 = pnp.element.mlnidentifier)
            
        return res
    
    def getWordInclusionMLN(self, textID):
        """Return the MLN for phrases that share intersect in their words without stop words."""
        
        res = '\n//Word inclusion\n\n'
        for sentence in self.sentences.itervalues():
            for np in sentence.getNPs():
                mentionLeafs = self.removeStopWordsFromTreeList([x for x in np.inOrderLeafs()])
                mentionWords = self.wordListFromNodeList(mentionLeafs)
                for asentence in self.sentences.itervalues():
                    for antenp in asentence.getNPs():
                        anteLeafs = self.removeStopWordsFromTreeList([x for x in antenp.inOrderLeafs()])
                        anteWords = self.wordListFromNodeList(anteLeafs)
                        if mentionWords <= anteWords and not len(mentionWords)==0 and not np==antenp:
                            res += 'wordInclusion({phrase1},{phrase2})\n'.format(phrase1=np.element.mlnidentifier, phrase2=antenp.element.mlnidentifier)
        return res
    
    def wordListFromNodeList(self, nodes):
        """Transform a list of Nodes to the List of Words of the elements."""
        
        result = [x.element.name for x in nodes]
        return set(result)
    
    def removeStopWordsFromTreeList(self, nodes):
        """Remove all elements in List of SyntacticTrees whose elements are in a StopList."""
        
        stopWords = self.getStopWords()
        for node in nodes:
            if node.element.name in stopWords:
                nodes.remove(node)
        return nodes   
    
    def getStopWords(self):
        this_dir, this_filename = os.path.split(__file__)
        DATA_PATH = os.path.join(this_dir, "div", "stopwords.txt")
        f = open(DATA_PATH, 'r')
        stopwords = [x.rstrip('\n') for x in f.readlines()] 
        return stopwords
    
    def getStrictHeadMatchingMLN(self,textID):
        """Check on several features for two phrases and if all apply create MLN feature."""
        
        res = '\n//Cluster Head Matches\n\n'
        headWords = self._getHeadWords()
        for headWord in headWords:
 #           for idx, sentence in self.sentences.iteritems():
            for aHeadWord in headWords:
                if aHeadWord.children[0].element.name == headWord.children[0].element.name:
                    aHeadWordNPs = self.findNPs(aHeadWord)
                    headWordNPs = self.findNPs(headWord)
                    for aHeadWordNP in aHeadWordNPs:
                        for headWordNP in headWordNPs:
                            if not headWordNP == aHeadWordNP:
                                res += 'clusterHeadMatch({headPhrase},{antecedentPhrase})\n'.format(headPhrase=headWordNP.element.mlnidentifier, antecedentPhrase=aHeadWordNP.element.mlnidentifier)
                            
                        
#                for np in sentence.tree.inOrderNPs():
#                    exists = False
#                    for leaf in np.inOrderLeafs():
#                        if headWord.children[0].element.name == leaf.element.name:
#                            exists = True
#                            break;
#                                
#                    if exists:
#                        #Get all the noun phrases the word belongs to
#                        headWordNP = self.findNPs(headWord)
#                        #Only interested in the first phrase as it is the one the word is a headword of
#                        antecedentNP = self.findNPs(leaf)[0]
#                        for headNP in headWordNP:
#                            res += 'clusterHeadMatch({headPhrase},{antecedentPhrase})\n'.format(headPhrase=headNP.element.mlnidentifier, antecedentPhrase=antecedentNP.element.mlnidentifier)
        return res
    
    def findNPs(self, word):
        """Return all Noun Phrases that are on the path to the root in the parse tree."""
        
        res = []
        if word.element.type == 'NP':
            res.append(word)
        if word.parent is not None:
            res += self.findNPs(word.parent)
        return res
    
    def find_phrases_on_path_to_root(self, phrase):
        """
        
        Return all phrases that are on the path to the root from the phrase.
        
        """
        res = []
        #Ignore the top level nodes as they create massive overhead
        ignore_list_patterns = ["^S_.+", "^ROOT-.+", "S-[0-9]+.+",
                                "^\..*", "^,.*"]
        ignore = False
        for pat in ignore_list_patterns:
            if not re.search(pat, phrase.element.identifier) is None:
                ignore = True
        if not ignore:
            res.append(phrase)
        if phrase.parent is not None:
            res += self.find_phrases_on_path_to_root(phrase.parent)
        return res
            
    def _getHeadWords(self):
        """ Return all the Head words of NPs in the text where the HW is the rightmost word of a NP."""
        
        headWords = []
        for idx, sentence in self.sentences.iteritems():
            for node in sentence.tree.inOrderNPs():
                if not len(node.children)==0:
                    i=1
                    flag = True
                    while flag:
                        if re.match('NN.*', node.children[-i].element.type):
                            headWords.append(node.children[-i])
                            flag = False
                        else:
                            i += 1
                            if i > len(node.children):
                                flag = False 
        
        return headWords
    
    def getAppositiveMLN(self,textID):
        """Return MLN features for the appositives of the text."""
        res = ''
        
        return res
    
    def getSentenceMLN(self,textID):
        result = ''
        result += "isaW(NULL,NULL)"
        for s in self.sentences:
            result += '\n\n//' + self.sentences[s].sentence + '\n'
            result += self.sentences[s].getMLN(s)
        return result
    
    def getDependenciesMLN(self,textID):
        result = '\n// Global Dependencies\n'
        role_words = set()
        special_chars = ['.',',','!','"','\'','@', '`','~',';','/','\\',
                         '[',']','{','}','(',')','$','%','&','#','^','-'
                         ,'_','=','+']
        for dep in self.dependencies:
            dep.sentences = self.sentences
            result += dep.getMLN()
            for template in dep.outputTemplate:
                if template.split("(")[0] == "hasRole":
                    role_words.update(dep.entities)
        for sentence in self.sentences.values():
            for word in sentence.tree.inOrderLeafs():
                if not word.element.mlnidentifier in role_words:
                    if word.element.identifier[0] in special_chars: continue
                    result += "hasRole({word}, NULL)\n".format(
                                                word = word.element.mlnidentifier)
        return result
    
    def getPhraseSimilarityMLN(self,textID):
        """Return all the phrasese that have all words in common."""
        #Filter special chars
        special_chars = ['.',',','!','"','\'','@', '`','~',';','/','\\',
                         '[',']','{','}','(',')','$','%','&','#','^','-'
                         ,'_','=','+']
        res = '\n\n// Phrase Similarity\n'
        allowedTags = ['NP']
        for idx, sentence in self.sentences.iteritems():
            for e in sentence.synelms:
                phraseSimilar = True
                for idx2, s2 in self.sentences.iteritems():
                    for e2 in s2.synelms:
                        # the length must match
                        similar = False
                        if len(e.words)==len(e2.words):
                            tmp = set()
                            tmp2 = set()
                            #if the word is encountered, set to true
                            for w in e.words:
                                #get all words to this phrase
                                subtree = self.parser.parseTree(sentence.tree, w, 'identifier')
                                #there should be only one subtree as we look for the identifier
                                kids = self.parser.getLeafElements(subtree[0], True)
                                if len(kids) == 0: tmp.add(w.rsplit('-',1)[0])
                                else:
                                    for k in kids:
                                        tmp.add(k.identifier.rsplit('-',1)[0])
                            for w2 in e2.words:
                                #get all words to this phrase
                                subtree = self.parser.parseTree(s2.tree, w2, 'identifier')
                                kids = self.parser.getLeafElements(subtree[0], True)
                                if len(kids) ==0: tmp2.add(w2.rsplit('-',1)[0])
                                else:
                                    for k in kids:
                                        tmp2.add(k.identifier.rsplit('-',1)[0])
                            if tmp==tmp2: similar = True
                            if similar and e.type in allowedTags and e2.type in allowedTags:
                                exp1 = '{P1}_S_{S1}'.format(P1=self.makeMLNRead(e.identifier),S1=idx)
                                exp2 = '{P2}_S_{S2}'.format(P2=self.makeMLNRead(e2.identifier),S2=idx2)
                                if not exp1 == exp2:
                                    if not e.identifier in special_chars or not e2.identifier in special_chars:
                                        res += 'phraseSimilarity({P1},{P2})\n'.format(P1 = exp1, P2 = exp2)    
        return res
    
    def makeMLNRead(self, exp):
        res = ut.helper.handleSpecialChars(exp)
        res = utils.toMLNCompatibleFormat(str(res))
        return res
    
    def getFilteredPhraseDistanceMLN(self, textID, blocked):
        #Filter special chars
        special_chars = ['.',',','!','"','\'','@', '`','~',';','/','\\',
                         '[',']','{','}','(',')','$','%','&','#','^','-'
                         ,'_','=','+']
        #take tree
        #parse tree until you find NP clause of List parser.parseTree(tree, 'NP')
        res = '\n\n// Phrases in the text\n'
        distanceVals = {0:'zerodistance', 1:'onedistance', 2:'twodistance', 3:'threedistance',4:'fardistance'}
        foundTrees = dict()
        for key in self.sentences.keys():
            foundTrees[self.sentences[key].sentenceID] = []
            for searchTerm in blocked:
                currentTree = self.parser.parseTree(self.sentences[key].tree, searchTerm)
                foundTrees[self.sentences[key].sentenceID].append(currentTree)
                for t in currentTree:
                    leafs = self.parser.getLeafElements(t, True)
                    for leaf in leafs:
                        res += 'isPhrase({word}_S_{sen},{phrase}_S_{sen})\n'.format(word=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(leaf.identifier)), phrase=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(t.element.identifier)), sen=str(self.sentences[key].sentenceID))
        res += '\n//Distance between the elements of the text\n'
        for key in foundTrees.keys():
            for type2 in foundTrees[key]:
                for tree in type2:
                    for key2 in foundTrees.keys():
                        for type in foundTrees[key2]:
                            for tree2 in type:
                                distance = abs(int(key) - int(key2))
                                if distance > 3: distance = 4
                                if not tree.element.name in special_chars or not tree2.element.name in special_chars:
                                    res += '{dis}({phrase1}_S_{S1},{phrase2}_S_{S2})\n'.format(dis=distanceVals[distance], phrase1=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(tree.element.identifier)), phrase2=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(tree2.element.identifier)), S1=key, S2=key2)
        
        #resturn all leafs of the node (the MLN represenation "isPhrase(word, phrase)") self.parser.getLeafElements(tree)
        #go to parent and continue traverse
        #After finish:
        #Get distances between all phrases
        return res
    
    def getText(self):
        raise NotImplementedError
        
    def getWordSimilarityMLN(self, textID):
        res = '\n\n// Word Similarity\n'
        for sentence in self.sentences:
            for key, wordEnt in self.sentences[sentence].words.iteritems():
                res += self.getWordPairSimilarityMLN(wordEnt, sentence)
        return res
    
    def getWordDistance(self, textID):
        res = ''
        for idx, sentence in self.sentences.iteritems():
            for leaf in self.sentences[idx].tree.inOrderLeafs():
                res += self.getWordDistanceMLN(leaf.element, 
                                               sentence.sentenceID)
        return res    

    def getPhraseDistanceMLN(self, textID, phraseList):
        res = ''
        for sentence in self.sentences:
            for syn in self.sentences[sentence].synelms:
                res += self._getPhraseDistanceMLN(syn, sentence, phraseList)
        return res
    
    def _getPhraseDistanceMLN(self, synElm, textID, phraseList):
        res = ''
        distanceVals = {0:'zerodistance', 1:'onedistance', 2:'twodistance', 3:'threedistance',4:'fardistance'}
        validphrases = ['ADJP','ADVP','CONJP','FRAG','INTJ','LST','NAC','NP','NX','PP','PRN','PRT','QP','RRC','UCP','VP','WHADJP','WHAVP','WHNP','WHPP','X']
        ignoredWords = ['\.', 'ROOT',',',';','S-'] 
        
        for idx, sentence in self.sentences.iteritems():
            for val in self.sentences[idx].synelms:
                distance = abs(int(idx)-int(textID))
                if distance > 4: distance = 4
                if isinstance(val, WordEntity) and isinstance(synElm, WordEntity):
                    pre = 'wtow'
                elif isinstance(val, SentenceChunk) and isinstance(synElm, SentenceChunk):
                    if not val.type in validphrases or self.stringContainsElements(ignoredWords, val.identifier) or self.stringContainsElements(ignoredWords, synElm.identifier):
                        continue
                    pre = 'ptp'
                elif isinstance(val, SentenceChunk) and isinstance(synElm, WordEntity):
                    pre = 'ptw'
                elif isinstance(val, WordEntity) and isinstance(synElm, SentenceChunk):
                    pre = 'twp'
                
                res += '{reltype}{dis}({word1}_S_{sw1}, {word2}_S_{sw2})\n'.format(reltype = pre, dis= distanceVals[distance],sw1=sentence.sentenceID, word1=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(val.identifier)), sw2=idx, word2=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(synElm.identifier)))
                
        return res
        
    def stringContainsElements(self, elements, exp):
        """Return true if any pattern is in exp."""
        for e in elements:
            if re.match(e, exp) is not None:
                return True
        return False
    
    def getWordPairSimilarityMLN(self, currentWord, currentSentence):
        res = ''
        #Filter special chars
        special_chars = ['.',',','!','"','\'','@', '`','~',';','/','\\',
                         '[',']','{','}','(',')','$','%','&','#','^','-'
                         ,'_','=','+']
        for sentence in self.sentences:
            for key, word in self.sentences[sentence].words.iteritems():
                r1 = 'root' == currentWord.word or 'Root' == currentWord.word or word.word == 'root' or word.word == 'Root'
                wsimilar = word.identifier == currentWord.identifier
                if not r1 and not wsimilar:
                    #Willkuerlich bei 0.8 den similarity score fuer eine gleicheit fesgelegt
                    sc = self.getWordSimilarityScore(currentWord.word, word.word)
                    if sc >= 0.8 and (not currentWord.word in special_chars and not word.word in special_chars):
                        res += "similar({W1}_S_{S1}, {W2}_S_{S2})".format(W1=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(currentWord.identifier)), W2=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(word.identifier)), 
                                                                          S1 = utils.toMLNCompatibleFormat(str(currentSentence)), 
                                                                          S2=utils.toMLNCompatibleFormat(str(sentence))) 
                        res += '\n'
        return res
    
    def getWordDistanceMLN(self, current_element, sentence_id):
        """Return the distance of one word to all other words in the text."""
        
        res = ''
        distances = {0:'distance0',
                     1:'distance1',
                     2:'distance2',
                     3:'distance3',
                     4:'fardistance'}
        for idx, sentence in self.sentences.iteritems():
            for leaf in self.sentences[idx].tree.inOrderLeafs():
                r1 = ('root' == current_element.name or (
                        'Root' == current_element.name or (
                        leaf.element.name == 'root' or ( 
                        leaf.element.name == 'Root'))))
                
                wsimilar=(leaf.element.identifier == current_element.identifier)
                if not r1 and not wsimilar:
                    dist = abs(int(sentence_id) - int(idx))
                    if dist > 3:
                        dist = 4
                    res += "{distance}({g1},{g2})\n".format(
                                            distance = distances[dist],
                                            g1 = current_element.mlnidentifier,
                                            g2 = leaf.element.mlnidentifier)
        return res
                     
    def getWordSimilarityScore(self, word1, word2):
        return jellyfish.jaro_winkler(word1, word2)

    def handleSpecialChars(self,word):
        tags = {'\$':'SDOLLAR','\.':'SPOINT'}
        res = word
        for key in tags.keys():
            res = re.sub(key,tags[key], res)
        return res

class SentenceEntity:
    
    """Manages the information for one sentence.
    
    Public Methods:
    __init__ --- Public constructor
    
    Public fields:
    sentence --- The sentence that this instance represents.
    dependencies --- List of dependencies between the words in this sentence.
    words --- List of words in this sentence.
    
    """
    
    def __init__(self, sentenceInfo, parser, entityDict=None, sentenceID = None):
        
        """Public constructor
        
        Key arguments:
        sentenceInfo --- A dictionary with all the information about a sentence.
        parser --- A syntactical parser.
        entityDict --- A dictionary to keep track of the entities of a sentence.
        
        """
        logging.info('Create a new Sentence')
        self.insert = Insertion()
        #List of all the words of the sentence
        self.sentence = str(sentenceInfo['sentence'])
        self.sentenceID = sentenceID
        #Get a list of all the words and all the dependencies between them
        self.dependencies, self.words, self.synelms, self.tree = self._getTags(sentenceInfo, self.sentence, parser, entityDict, sentenceID)

    def saveSentenceToSQLite(self, sqlTextID, cursor):
        """
        Save this sentence to SQLite.
        This includes the tree, the words, lemma and word_instances,
        definitions and examples and senses
        """
        

        sentenceID = self.insert.insert_or_update_sentence(self.sentenceID, 
                                                        sqlTextID, 
                                                        self.sentence, 
                                                        cursor=cursor)
        # phrases to db
        phrases = dict()
        words = dict()
        senses = dict()
        lemmas = dict()
        objectEntities = dict()
        
        for phrase in self.tree.inOrderPhrases(leafs = False):
            phrases[phrase.element.identifier] = (
                self.insert.insert_or_update_phrase(phrase.element.identifier,
                                                  sqlTextID, sentenceID,
                                                  phrase.element.type,
                                                  cursor=cursor))
            if phrase.parent is not None:
                if not (phrase.parent.element.identifier in phrases):
                    phraseID = self.insert.insert_or_update_phrase(
                                            phrase.parent.element.identifier,
                                            sqlTextID, sentenceID,
                                            phrase.parent.element.type,
                                            cursor=cursor)
                                            
                self.insert.insert_or_update_phrase_phrase(
                                    phrases[phrase.parent.element.identifier],
                                    phrases[phrase.element.identifier],
                                    cursor=cursor)
                
            if (not (phrase.element.identifier in objectEntities.keys()) 
                    and phrase.element.groundIdentifier is not None):
                objectEntities[phrase.element.groundIdentifier] = (
                        self.insert.insert_or_update_grounding(
                                phrases[phrase.element.identifier],
                                phrase.element.groundIdentifier, cursor=cursor))
            
            elif phrase.element.groundIdentifier is not None:
                gi = self.insert.insert_or_update_object(
                                phrase.element.groundIdentifier, cursor=cursor)
                self.insert.insert_or_update_grounding(
                        phrases[phrase.element.identifier], gi, cursor=cursor)
                
                                                                   
                    
            
        #Words to DB

        for word in self.words.itervalues():
            if not (word.majoritySenseEntity == "NONE") and (
                                        word.majoritySenseEntity is not None):  
                senses[word.majoritySenseEntity._senseID] = ( 
                    self.insert.insert_or_update_sense(
                                            word.majoritySenseEntity._senseID,
                                            cursor=cursor))
            words[word.index] = self.insert.insert_or_update_word(word.word,
                                                                cursor=cursor)
            
        
        for word in self.words.itervalues():
            if (word.majoritySenseEntity is not None) and not (
                                            word.majoritySenseEntity == "NONE"):
                lemma = self.findLemma(word.majoritySenseEntity._senseID, word)
                if not lemma in lemmas.keys():
                    lemmaID = Insertion.insert_or_update_lemma(
                                                        lemma, cursor=cursor)
                    lemmas[lemma] = lemmaID
            else:
                lemma = 0
            if not lemma in lemmas.keys():
                currentLemma = 0
            else:
                currentLemma = lemmas[lemma] 
            syntacticObject = 0
            if not word.majoritySenseEntity == "NONE":
                currentSense = senses[word.majoritySenseEntity()._senseID]
            else:
                currentSense = ""
            if word.syntacticObject is not None:
                syntacticObject = phrases[word.syntacticObject.identifier]
            self.insert.insert_or_update_word_instance(words[word.index],
                                        sentenceID, 
                                        sqlTextID, 
                                        word.index, 
                                        syntacticObject, 
                                        word.POS,
                                        currentLemma,
                                        currentSense, 
                                        cursor=cursor)
        
        for dep in self.dependencies:
            self.insert.insert_or_update_dependency(words[dep.first.index],
                                                  words[dep.second.index],
                                                  dep.typ, sqlTextID,
                                                  sentenceID, cursor=cursor)
    
    def findLemma(self, synset, word):
        """
        Find the lemma for a word given a synset.
        """
        
        lemmas = wn.lemmas(word)
        for lemma in lemmas:
            if lemma.synset == synset:
                return lemma
    
    def getMLN(self,sentenceID):
        """return the MLN representation of a sentence."""
        #print the MLN for the words
        result = "\n//Dependencies\n"
        for dep in self.dependencies:
            result += "%s(%s_S_%s,%s_S_%s)\n"%(dep.typ, 
                                        dep.first.identifier.strip('\''), 
                                        utils.toMLNCompatibleFormat(sentenceID), 
                                        dep.second.identifier.strip('\''), 
                                        utils.toMLNCompatibleFormat(sentenceID))
        
        for word in self.words:
            result += self.words[word].getMLN(sentenceID)
        
        for se in self.synelms:
            result += se.getMLN(sentenceID)

        return result
        
    def _splitStanfordParserDependency(self, dep):
        
        """Split the dependency output of the Stanford parser.
        
        Key arguments:
        dep ---  The output for one dependency of the Stanford parser.
        
        """
        
        first = re.split('\(', dep.toString())
        second = re.split(',', first[1])
        third = re.split('\)', second[1])
        return [first[0], second[0], third[0]]

    def _getEntityObjectRef(self, word, entityDict):
        
        """Return the reference to an object or create a new object and return it.
        
        Key arguments:
        identifier --- Unique identifier of the entity Object.
        entityDict --- Global dictionary of all entities seen so far.
        
        """
        
        if not word.syntacticObject is None and word.syntacticObject.identifier in entityDict:
            return entityDict[word.syntacticObject]
        #uncomment if all words should be initialized with an entity
        #elif word.syntacticObject is None:
            #currentIdent = word.word + '-' + str(len(entityDict))
            #entityDict[currentIdent] = SentenceChunk(word.word + '-' + str(len(entityDict)))
            #entityDict[currentIdent].words.add(word)
            #return entityDict[currentIdent]
        return None
    
    def getNPs(self):
        nps = self.tree.inOrderNPs()
        return nps
    
    def getPhrases(self):
        return self.tree.inOrderPhrases()
    
    def _getWordDetails(self, sentenceInfo):
        
        """Get a dictionary of words with all their details that are provided in the sentenceInfo.
        
        Key arguments:
        sentenceInfo --- A dictionary with all the information about a sentence.
        
        """
        
        wordEntities = dict()
        if 'words' in sentenceInfo:
            for w in sentenceInfo['words']:
                wordEntities[int(w['index'])] = dict()
                if 'index' in w:
                    wordEntities[int(w['index'])]['index']= int(w['index'])
                if 'senses' in w:
                    wordEntities[int(w['index'])]['senses'] = w['senses']
                if 'roles' in w:
                    wordEntities[int(w['index'])]['roles'] = w['roles']
                if 'entity' in w:
                    wordEntities[int(w['index'])]['entity'] = w['entity']
        else:
            return None
        return wordEntities
    
    def _createWordsList(self, pos, sentenceInfo, entityDict, wordList, sentenceID):
        
        """Create a list of Words with the Part of Speech tags and add senses, roles, entity reference.
        
        Key arguments:
        pos --- A list with the results of the Stanford Part of Speech tagging.
        sentenceInfo --- A dictionary with all the information about a sentence.
        entityDict --- Dictionary that is used to keep track of all entities in one Text.
        
        """
        
        words = dict()
        words[0] = WordEntity(0, "root")
        
        #make a dictionary with entities and sensens and roles for the words
        wordEntities = self._getWordDetails(sentenceInfo) 
            
        for w in wordList:
            splittedWord = str(w.name)
            logging.info('Add Word %s to the list.', w.name )
            id = int(w.identifier.rsplit('-', 1)[1])
            r = None
            s = None
            e = None
            if not wordEntities is None and id in wordEntities:
                if 'roles' in wordEntities[id]:
                    r = wordEntities[id]['roles']
                if 'senses' in wordEntities[id]:
                    s = []
                    for sense in wordEntities[id]['senses']:
                        s.append(SenseEntity(sense['sense'], sense['votes']))
                if 'entity' in wordEntities[id]:
                    e = wordEntities[id]['entity']
            words[id] = WordEntity(id, 
                                   str(w.name),
                                   str(w.pos),
                                   role = r,
                                   sense = s,
                                   entityObject = None)
            words[id].sentence = sentenceID
        return words
    
    def _getTags(self, sentenceInfo, sentence, parser, entityDict, sentenceID):
        
        """Get all the tags of one word.
        
        Key arguments:
        sentenceInfo --- A dictionary with all the information about a sentence.
        entityDict --- Dictionary that is used to keep track of all entities in one Text.
        sentence --- The sentence to be parsed.
        parser --- A stanford parser.
        
        """
        
        deps, pos, ses, wordList, tree = self._getPosAndDepsAndSEs(sentence, parser, sentenceID)
        #Create the list of words
        words = self._createWordsList(pos, sentenceInfo, entityDict, wordList, sentenceID)
        #Create dependencies
        dependencies = self._createDependencies(deps,words)
        #Create all the syntactic elements of the sentence
        synelms = self._createSyntacticElements(ses, words, tree, sentenceID)
        return dependencies, words, synelms, tree
    
    def _createSyntacticElements(self, ses, words, tree, sentenceID):
        """Create all Syntactic Elements relations present in the sentence."""
        res = []
        tmp = SentenceChunk(tree.element.identifier, tree.element.type, sentenceID)
        
        
        if len(tree.children) == 0 or tree.children is None:
            tmp.words.add(words[int(tree.element.identifier.split('-')[-1])])
            return res
        
        for child in tree.children:
            tmp.words.add(child.element.identifier)
            res += self._createSyntacticElements(ses, words, child, sentenceID)
        res.append(tmp)    
#        for s, value in se.iteritems():
#            for idx, val in enumerate(value):
#                tmp = SentenceChunk(val, s, sentence = sentenceID)
#                for v in value[val]:
#                    if v.name is not None:
#                        tmp.words.add(words[int(v.identifier.split('-')[-1])])
#                        continue
#                    tmp.words.add(v)
#                res.append(tmp)
        return res

    def _createDependencies(self,deps,words):
        
        """
        Create Dependency Objects.
        
        Key arguments:
        deps --- List of dependencies as from the Stanford Parser output.
        words --- Dictionary of words.
        """
        
        result = []
        for dep in deps:
            logging.info('Add dependency: %s', dep)
            result.append(self._SFDfromParser(dep,words))
        return result

    def _matchWordtoEntity(self, w, words):
        
        """Return the actual word object from the words list given a word in format 'word-index'.
        
        Key arguments:
        w --- The word to be split.
        words --- The dictionary of words of the sentence.
        
        """
        
        return words[int(str(w).rsplit('-',1)[1])]
    
    def _SFDfromParser(self, dep, words):
        
        """Create a StanfordDependency object.
        
        Key arguments:
        dep --- The output of the StanfordParser for one dependency.
        words --- The words in the sentence.
        
        """
        
        type, w1, w2 = self._splitStanfordParserDependency(dep)
        source = self._matchWordtoEntity(w1,words)
        destiny = self._matchWordtoEntity(w2,words)
        result = StanfordDependency(type,source,destiny)
        return result
    
    def _getPosAndDepsAndSEs(self, sentence, parser, sentenceID):
        
        """Parse a sentence and get the Part of Speech tags as well as dependencies.
        
        Key arguments:
        sentence --- The sentence.
        parser --- A syntactic parseer.
        
        """
        
        logging.info('get POS and Dependencies from parser for sentence: "' + sentence + '"')
        result = parser.getPOSandDep(sentence, collapsed=True)
        result = result + parser.getSEs(sentenceID, sentence)
        return result
    
    
class StanfordDependency:
    """Represent one dependency.
    
    Public Methods:
    __init__ --- Public constructor
    
    Public fields:
    type --- The type of the dependency.
    first --- The first part of the dependency.
    second --- The second part of the dependecy.
    
    """
    
    def __init__(self, typ, first, second):
        """The public constructor.
        
        Key argumenst:
        typ --- The type of the dependency.
        first --- The first part of the dependency.
        second --- The second part of the dependecy.
        
        """
        
        self.typ = typ
        self.first = first
        self.second = second
        
class DepedencyEntity:
    
    def __init__(self, typ, entities, output):
        self.type = typ
        #the entities involved
        self.entities = entities
        #The template for writing the MLN output for the dependency
        self.outputTemplate = output
        
        self.dbInsert = Insertion()
        self.sentences = None
    
    @property
    def identifier(self):
        if len(self.outputTemplate) > 1:
            outputString = self.getMLN().split("\n")[0]
        else:
            outputString = self.getMLN()
        return outputString
    
    def getMLN(self, functional = True):
        res = ''
        for template in self.outputTemplate:
            res += self.parseTemplate(template, self.entities, 
                                      functional = functional)
            res += '\n'
        return res
    
    def saveToSQLite(self, text, cursor):
        """
        Save Dependency to SQLite Database.
        """
        
        depID = self.dbInsert.insert_or_update_dependency_entity(self.type, 
                                                                text, 
                                                                cursor = cursor) 
        for phrase in self.entities:
            parts = phrase.split('_')
            ident = "{typ}-{number}".format(typ=parts[0],number=parts[1])
            sentence = parts[3]
            
            phraseID = self.dbInsert.insert_or_update_phrase(ident, 
                                                             text, 
                                                             sentence, 
                                                             parts[0], 
                                                             cursor=cursor)
            
            self.dbInsert.insert_or_update_depentity_phrase(depID, 
                                                            phraseID, 
                                                            cursor=cursor)
        
    def parseTemplate(self,template, entities, functional = True):
        result = template
        tmp = list(entities)
        for match in re.finditer('{E[0-9]+}', template):
            result = re.sub(match.group(), tmp.pop(0), result, 1)

        return result
            

        
class WordEntity:
    
    """One word that holds its sense, roles, a reference to the entity and POS.
    
    Public methods:
    __init__ --- Public constructor
    
    Public fields:
    index --- The index of the word in the sentence it stands.
    word --- The word as a string.
    POS --- The Part of Speech of the Word.
    sense --- A list of senses that the word has.
    role --- The roles the word has.
    syntacticObject --- A reference to the entity this word represents.
    
    """
    
    def __init__(self, index ,word, pos = None, sense = None, role=None, 
                 entityObject = None):
        
        """Public constructor.
        
        Key arguments:
        index --- The index of the word in the sentence it stands.
        word --- The word as a string.
        POS --- The Part of Speech of the Word.
        sense --- A list of senses that the word has.
        role --- The roles the word has.
        syntacticObject --- A reference to the entity this word represents.
        
        """
        
        self.index = index
        self.word = word
        self.POS = pos
        self._sense = sense
        self.role = role
        self.syntacticObject = entityObject
        self._settedSense = None
        self.sentence = None
        self.manual_word_representations = []
    
    @property
    def possibleSenses(self):
        """Return all senses wordnet has for this word"""
        senses = wn.synsets(self.word)
        for w in self.manual_word_representations:
            senses = senses + wn.synsets(w)
        res = []
        if senses is not None:
            for s in senses:
                res.append(SenseEntity(s.name, 0, s.definition, s.examples))
            return res
        return []
    
    @property
    def identifier(self):
        return utils.toMLNCompatibleFormat(self.word) + "_" + str(self.index)

    @property
    def majoritySense(self):
        if self._settedSense is not None:
            return self._settedSense._senseID 
        nltkID = self._majoritySense()
        nltkID = utils.toMLNCompatibleFormat(nltkID)
        return nltkID
    
    @property
    def majorityRawSense(self):
        nltkID = self._majoritySense()
        return nltkID
    
    @property
    def majoritySenseEntity(self):
        if self._settedSense is not None:
            return self._settedSense
        maxVote = 0
        res = None
        #print self.word
        if self._sense is None:
            return 'NONE'
        for s in self._sense:
            if s.votes > maxVote:
                res = s
                maxVote = s.votes
        return res
    
    def getMLN(self, sentence, possibleSenses=False):
        #Filter special chars
        special_chars = ['.',',','!','"','\'','@', '`','~',';','/','\\',
                         '[',']','{','}','(',')','$','%','&','#','^','-'
                         ,'_','=','+']
        if self.word in special_chars:
            return ""
        
        if self.word == 'root':
            return ""
        result = '\n//'+self.word+'\n'
        result += "hasPOS(%s_S_%s, %s)\n"%(utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(self.identifier)), utils.toMLNCompatibleFormat(sentence), ut.helper.handleSpecialChars(self.POS))
        if not self.role is None:
            for r in self.role:
                result += "hasRole(%s_S_%s, %s)\n"%(utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(self.identifier)), utils.toMLNCompatibleFormat(sentence), r['role'])
                #result += "hasRole(%s, %s)\n"%(self.identifier, r['role'])
        """if not self.syntacticObject is None:
            result += "isEntity(%s_S_%s, %s)\n"%(self.identifier, utils.toMLNCompatibleFormat(sentence),
                    utils.toMLNCompatibleFormat(self.syntacticObject.identifier))"""
        if not self._sense is None or not self._settedSense is None or not self._sense == 'NOMATCH' or not self._sense == 'OTHERTYPE':
            #result += "isa(%s,%s)\n"%(self.identifier, self.majoritySense)
            if(isinstance(self.majoritySenseEntity, SenseEntity)):
                result += "hasSense({word}_S_{sentence}, {sense})\n".format(
                                word=utils.toMLNCompatibleFormat(
                                                ut.helper.handleSpecialChars(
                                                            self.identifier)),
                                sentence=utils.toMLNCompatibleFormat(sentence), 
                                sense=utils.toMLNCompatibleFormat(
                                        self.majoritySenseEntity.nltkSense))
                result += "isaW({sense}, {sense})\n".format(
                                word=utils.toMLNCompatibleFormat(
                                                ut.helper.handleSpecialChars(
                                                            self.identifier)),
                                sentence=utils.toMLNCompatibleFormat(sentence), 
                                sense=utils.toMLNCompatibleFormat(
                                        self.majoritySenseEntity.nltkSense))
            
            else:
                result+= "hasSense({word}_S_{sentence}, NULL)".format(
                            word=utils.toMLNCompatibleFormat(
                                                ut.helper.handleSpecialChars(
                                                        self.identifier)),
                            sentence = utils.toMLNCompatibleFormat(sentence))
            pathSet = set()
            if not self.majoritySenseEntity=='NONE':
                for s in self.majoritySenseEntity.sensePath:
                    pathSet.update(s)
                for s in pathSet:
                    result += "isaW(%s, %s)\n"%(utils.toMLNCompatibleFormat(self.majoritySenseEntity.nltkSense),
                                                 utils.toMLNCompatibleFormat(s.name))
        if possibleSenses:  result += self.possibleSensesMLN(sentence)
        return result
        
    def possibleSensesMLN(self, sentence):
        res = "\n// Possible Senses\n"
        for sense in self.possibleSenses:
            res += "\nhasPossibleSense({word}_S_{sentence}, {sense})\n".format(
                                        word=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(self.identifier)),
                                        sentence = utils.toMLNCompatibleFormat(sentence),
                                        sense=utils.toMLNCompatibleFormat(sense.nltkSense))
            pathSet = set()
            map(pathSet.update, sense.sensePath)
            for s in pathSet:
                res += "isaW({sense}, {concept})\n".format(
                                sense=utils.toMLNCompatibleFormat(
                                                            sense.nltkSense), 
                                concept=utils.toMLNCompatibleFormat(
                                                            s.name))
                        
        res += "hasPossibleSense({word}_S_{sentence}, {sense})\n".format(
                                        word=utils.toMLNCompatibleFormat(ut.helper.handleSpecialChars(self.identifier)),
                                        sentence = utils.toMLNCompatibleFormat(sentence),
                                        sense="NULL")
        return res
    
    
    def _majoritySense(self):
        if self._settedSense is not None:
            return self._settedSense._senseID
        maxVote = 0
        res = None
        if self._sense is None:
            return None
        for s in self._sense:
            if s.votes > maxVote:
                res = s.nltkSense
                maxVote = s.votes
        return res
    
    def setSense(self,senseID):
        ps = self.possibleSenses
        if ps is None:
            self._settedSense = None
            return ""
        for s in ps:
            if s._senseID == senseID:
                self._settedSense = s
                return
        return

class SentenceChunk:
    
    """Resembles one Object that many words can refer to.
    
    Public methods:
    __init__ --- Public constructor.
    
    Public fields:
    identifier --- The unique identifier that is used to identify the object.
    
    """
     
    def __init__(self,identifier, type = None, sentence = None):
        
        """The public constructor.
        
        Key arguments:
        identifier --- The unique identifier that is used to identify the object.
        
        """
        self.tags = ['ADJP','ADVP','CONJP','FRAG','INTJ','LST','NAC',
                     'NP','NX','PP','PRN','PRT','QP','RRC','UCP','VP',
                     'WHADJP','WHAVP','WHNP','WHPP','X', 'S','SBAR','SBARQ',
                     'SINV','SQ','CC','CD','DT','EX','FW','IN','JJ','JJR',
                     'JJS','LS','MD','NN','NNS','NNP','NNPS','PDP','POS',
                     'PRP','PRP$','RB','RBR','RBS','RP','SYM','TO','UH',
                     'VB','VBD','VBG','VBN','VBP','VBZ','WDT','WP','WP$','WRB']
        self.identifier = identifier
        self.sentence = sentence
        self.type = type
        self.words = set()
        
    def getMLN(self, sentenceID):
        res = ""
        for se in self.words:
            tmp = 'wordphrase'
            if se.rsplit('-',1)[0] in self.tags:
                tmp = 'synphrase'
            vals, tmp = self.getValMLN(se, sentenceID)
        return res
            
    def getValMLN(self, w, sentenceID):
        res = ""
        tmp = 'wordphrase'
        if w.rsplit('-',1)[0] in self.tags:
                tmp = 'synphrase'
        if isinstance(w, unicode):
            res += utils.toMLNCompatibleFormat(
                                    ut.helper.handleSpecialChars(
                                    str(w))) + '_S_' + str(sentenceID) + ', '
        if isinstance(w, Trees.SyntacticElement):
            res += utils.toMLNCompatibleFormat(utils.toMLNCompatibleFormat(
                                        ut.helper.handleSpecialChars(
                                        w.identifier)) + "_S_" + str(
                                                            sentenceID)) + ' ,'

        #remove the last comma
        res = res[0:len(res)-2]
        return [res, tmp]
            
        
class SenseEntity:
    def __init__(self, senseID, votes, definition = None, examples = []):
        self._senseID = senseID
        self.votes = votes
        self.definition = definition
        self.examples = examples
    
    """Get the MLN representation of the path of the synset"""
    @property
    def pathMLN(self):
        pass
        
    @property
    def nltkSense(self):
        return self.wordnetSynsetToNLTK(self._senseID)
    
    @nltkSense.setter
    def nltkSense(self,value):
        self._senseID = value
    
    @property
    def sensePath(self):
        if len(self._senseID.split('-'))<3:
            return wn.synset(self._senseID).hypernym_paths()
        thing = wn.synset(self.nltkSense)
        return thing.hypernym_paths()
    
    @property
    def sensePathNLTK(self):
        thing = wn.synset(self._senseID)
        return thing.hypernym_paths()

    def wordnetSynsetToNLTK(self,wordnetID):
        if wordnetID == '':
            return wordnetID
        elif wordnetID == "NOMATCH":
            return "NOMATCH"
        elif wordnetID == "OTHERTYPE":
            return "OTHERTYPE"
        elif len(wordnetID.split('.'))>0:
            return wordnetID
        cleanID = (wordnetID[4:]).lstrip('0').rpartition("-")
        pos = cleanID[2].lower()
        offset = cleanID[0]
        nltkID = str(wn._synset_from_pos_and_offset(pos, int(offset)))
        tmpID = re.search('\'.+\'', nltkID)
        nltkID = tmpID.group().strip('\'')
        return nltkID
                    
if __name__ == "__main__":
    import jpype
    import os
    import cProfile
    from StanfordParser import *
    
    classpath = [os.path.join('..', '..', '3rdparty', 
                              'stanford-parser-2012-02-03', 
                              'stanford-parser.jar')]
    print jpype.getDefaultJVMPath()
    jpype.startJVM(jpype.getDefaultJVMPath(), '-ea', 
                   '-Djava.class.path=%s' % (':'.join(classpath)))
    parser = Parser(os.path.join('..','..','3rdparty',
                                 'stanford-parser-2012-02-03',
                                 'grammar','englishPCFG.ser.gz'))
    sm = SentenceManager(parser)
    #cProfile.run(statement='sm.loadJsonFile("/usr/stud/meyer/Source/semcore_annotation_tool/src/hits.results")',filename='out.profile')
    #cProfile.run(statement='sm.loadJsonFile("/usr/stud/meyer/Source/data/live_2/ADD/HITS.results")',filename='out.profile')
    print "All Loaded"
    jpype.shutdownJVM()
