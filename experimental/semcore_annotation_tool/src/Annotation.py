# Class that resembles an entire annotation for one sentence
# This includes the output of the stanfort parser, the Senes from the Mechanical Turk Project and roles assigned in this tool

#Imports
from StanfordParser import *
import os
import re
import utils
from JSON2DB import *
from prac_nltk.corpus import wordnet as wn
from prac_nltk.corpus import verbnet as vn
import copy
from copy import deepcopy


class Annotation:
    
    def __init__(self, sentence, sentenceID):
        self.sentence = sentence
        self.words = []
        self.deps = []
        self.sentenceID = sentenceID
        self.sentenceARR = []
        self.output = ""
        self.indexOffset = 0 
    
    #Add the word senses from the Mechanical Turk project
    def addSenses(self):
        self.output += '\n// Word Senses from Mechanical Turk\n'
        self.output += '\n'.join([str(x) for x in self.currAnnot])
    
    #Make the parser output compatible to MLN
    def parserOutPutToMLNCompatible(self,x):
        source = x.toString()
        resarr = re.split('(\(.+\))',source)
        resarr = re.split(', ', resarr[1])
        arg1 = resarr[0].strip('(')
        arg2 = resarr[1].strip(')')
        source = re.sub(arg1,utils.toMLNCompatibleFormat(str(arg1)),source)
        source = re.sub(arg2,utils.toMLNCompatibleFormat(str(arg2)) + ",S" + str(self.sentenceID),source)
        return source
    
    #Add a word to this sentence
    def addWord(self, index, result, tag, context, word):
        self.words.append(Word(index, tag, context, result, word))
        return self.words[len(self.words)-1]

    #get the result of the Stanford parser as a String that can be printed
    def getDependenciesAsString(self):
        pass

    #Return all the senses in a string format that can be printed
    def getSenesAsString(self):
        pass

    def tagSense(self):
        return
    
    def tagPOS(self):
        return
    
class Word:
    def __init__(self, index, tag, context, result, word):
        self.word = word
        #Index is 1 less than the index used by the Stanford Parser
        self.stanfordDep = None
        self.index = index
        self.tag = tag
        self.context = context
        self.result = result
        self.roles = []
        self.selectedSense = 0
        #Fill with the path Synsets
        self.path = []
    
    def addRole(self, r):
        #Check if role has been added already
        for role in self.roles:
            if role == r:
                return False
        #If role ist new -> Add to word
        self.roles.append(r)
        return True
    
    def deleteRole(self,r):
        try:
            self.roles.remove(r)
        except ValueError:
            return False
    
    def deleteAllRoles(self):
        self.roles = []
        
    def addRoles(self, roles):
        self.roles = roles
        
    #Uses NLTK to transform a Wordnet ID to a NLTK ID that is MLN conform
    def _wordNetToNLTK(self,wordnetID, raw=False):
        if wordnetID == '':
            return wordnetID
        elif wordnetID == "NOMATCH":
            return "NOMATCH"
        elif wordnetID == "OTHERTYPE":
            return "OTHERTYPE"
        cleanID = (wordnetID[4:]).lstrip('0').rpartition("-")
        pos = cleanID[2].lower()
        offset = cleanID[0]
        nltkID = str(wn._synset_from_pos_and_offset(pos, int(offset)))
        tmpID = re.search('\'.+\'', nltkID)
        nltkID = tmpID.group().strip('\'')
        #Make MLN conform
        if raw:
            return nltkID
        nltkID = utils.toMLNCompatibleFormat(nltkID)
        return nltkID
    
        
    #Get the sense with the most votes
    def getMajoritySense(self, senses):
        if senses is None:
            return False
        maxvotes = 0
        sense = None
        for r in senses:
            v = r['votes']
            if v > maxvotes:
                maxvotes = v
                sense = r['sense']
        return self._wordNetToNLTK(sense)
    
    #Return an ordered List of all the sense
    def getSenseList(self):
        if self.result is None:
            return False
        senses = self.result
        #have to get around making a deep copy of the path
        sortedSenses = sorted(senses, key = lambda x : x['votes'],reverse=True)
        tmpL= []
        tmpPaths = []
        for en in sortedSenses:
            tempDict = {'sense':en['sense'],'votes':en['votes']}
            tmpPaths.append(en['path'])
            tmpL.append(tempDict)
            senses_sorted = deepcopy(tmpL)
            i = 0
            for sense in senses_sorted:
                sense['path'] = tmpPaths[i]
                i = i+1
        
        #    senses_sorted = deepcopy(sortedSenses) 
        for x in senses_sorted:
            x['sense'] = self._wordNetToNLTK(str(x['sense']))
        return senses_sorted
    
        #Make the parser output compatible to MLN
    def _parserOutPutToMLNCompatible(self,x):
        source = x.toString()
        resarr = re.split('(\(.+\))',source)
        resarr = re.split(', ', resarr[1])
        arg1 = resarr[0].strip('(')
        arg2 = resarr[1].strip(')')
        source = re.sub(arg1,utils.toMLNCompatibleFormat(str(arg1)),source)
        source = re.sub(arg2,utils.toMLNCompatibleFormat(str(arg2)) + ",S" + str(self.sentenceID),source)
        return source
    
class StanfordDependency:
    def __init__(self, type, origin, destiny):
        self.type = type
        self.origin = origin
        self.destiny = destiny
