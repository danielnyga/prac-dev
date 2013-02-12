#Sentence Manager manages all sentences that are annotated

from Annotation import *
from FileManager import *
from JSON2DB import *
import os
from Tkinter import *
import re

class SentenceManager:
    
    def __init__(self):
        #Current index is just length of this List
        self.fileManager = FileManager()
        self.parser = Parser(os.path.join('..','..','3rdparty','stanford-parser-2012-02-03','grammar','englishPCFG.ser.gz'))
        self.annotationList = []
        #self.loadJsonFile(os.path.join('..','..','data','live_2','ADD','HITS.results'))
    
    def _makeAllAnnotationsMLNCompatible(self, annoList):
        for anno in annoList:
            self._makeAnnnotationMLNCompatible(anno)
            
    #Parses the JSON directly into the annotation List of this class
    def jsonParser(self, obj):
        sublist = True
        for key in obj.keys():
            if key == 'sentence':
                sublist = False
        if sublist:
            return obj
        if len(self.annotationList) == 0:
            #Only to test
            self.annotationList.append(Annotation(obj['sentence'], 0))
            self.annotationList[len(self.annotationList) - 1].addWord(0, None, None, obj['context'], "Root")
        for entry in self.annotationList:
            #Check if first occurance of the sentence
            if entry.sentence == obj['sentence']:
                entry.addWord(obj['index'] + 1, obj['results'], obj['tag'], obj['context'], re.sub(" ","-",obj['word']))
                return obj
        
        #Append a new Annotation to the annotation list
        self.annotationList.append(Annotation(obj['sentence'], len(self.annotationList)))
        newAnno = self.annotationList[len(self.annotationList) - 1]
        #newAnno = Annotation(obj['sentence'], len(self.annotationList))
        #Add a root word so we can match it to the output of the stanford parser
        newAnno.addWord(0, None, None, obj['context'], "Root")
        newAnno.addWord(obj['index'] + 1, obj['results'], obj['tag'], obj['context'], re.sub(" ","-",str(obj['word'])))
      
    
    def getPaths(self,word):
        if word.result is None:
            return
        for res in word.result:
            wordnetID = word._wordNetToNLTK(res['sense'], True)
            if wordnetID == '':
                res['path'] = []
                continue
            elif wordnetID == "NOMATCH":
                res['path'] = []
                continue
            elif wordnetID == "OTHERTYPE":
                res['path'] = []
                continue
            thing = wn.synset(wordnetID)
            paths = thing.hypernym_paths()
            res['path'] = paths

    #To each sentence, add all dependencies from the stanford parser
    def addAllDependencies(self, annotations):
        for anno in annotations:
            sentence = self.treatDoubleWords(anno.sentence, anno.words)
            deps = self.getDependencies(sentence)
            for dep in deps:
                splittedDependency = self.splitStanfordParserDependency(dep)
                self.addStanfordDependencyToAnnotation(splittedDependency, anno)
                #print 'test'
   
    #Add the dependencies to the annotation
    def addStanfordDependencyToAnnotation(self, sp, anno):
        #map the indexes to the word indexes from the JSON File
        # Form: type, source, origin
        depAndIndexes = self.getSDIndexes(sp)
        dep = StanfordDependency(depAndIndexes[0], None, None)
        for word in anno.words:
            splitted = word.word.split('-')
            if len(splitted) > 1:
                joinedSplitted = " ".join(splitted)
                if joinedSplitted in anno.sentence:
                    anno.sentence = re.sub(joinedSplitted, word.word, anno.sentence)
        
        POS = self.parser.getPOS(anno.sentence)
        origin = False
        destiny = False

        for word in anno.words:
            if word.index == int(depAndIndexes[1][1]):
                dep.origin = word
                origin = True
                if word.word=='Root':
                    continue
                word.tag = POS[word.index][1]
                #dep.origin = depAndIndexes[1][0]
            elif word.index == int(depAndIndexes[2][1]):
                destiny = True
                dep.destiny = word
                if word.word=='Root':
                    continue
                word.tag = POS[word.index][1]

        if not origin:
            #Also add as origin!
            w = anno.addWord(int(depAndIndexes[1][1]), None, None, None, str(depAndIndexes[1][0]).lstrip())
            if not w.word == "Root":
                w.tag = POS[w.index][1]
            dep.origin = w
        if not destiny:
            #Also add as destiny!
            w = anno.addWord(int(depAndIndexes[2][1]), None, None, None, str(depAndIndexes[2][0]).lstrip())
            if not w.word == "Root":
                w.tag = POS[w.index][1]
            dep.destiny = w
        anno.deps.append(dep)
    
    #Get the index from the dependency string                
    def getSDIndexes(self, sp):
        result = []
        result.append(sp[0])
        tmp = sp[1].rpartition('-')
        result.append([tmp[0], tmp[2]])
        tmp = sp[2].rpartition('-')
        result.append([tmp[0], tmp[2]])
        return result
        
    #Split the result of the stanford parser into its parts
    def splitStanfordParserDependency(self, dep):
        first = re.split('\(', dep.toString())
        second = re.split(',', first[1])
        third = re.split('\)', second[1])
        return [first[0], second[0], third[0]]
        
    def treatDoubleWords(self,sentence, words):
        for w in words:
            if w.word.count("-") > 0:
                sentence = re.sub(re.sub("-"," ",w.word),w.word, sentence)
        return sentence
    
    #Execute the Stanford parser                
    def getDependencies(self, sentence):
        #self.output += '\n// Syntactic Features from Stanford Parser\n'
        deps = self.parser.getDependencies(sentence)
        return deps
    
    def loadJsonFile(self, f):
        jsonfile = open(f, 'r')
        json.load(jsonfile,object_hook=self.jsonParser)
        self.correctIndexes()
        self.addAllDependencies(self.annotationList)
        self.addAllPaths(self.annotationList)
        self._makeAllAnnotationsMLNCompatible(self.annotationList)
        return self.annotationList
    
    def addAllPaths(self,annoList):
        for anno in annoList:
            for word in anno.words:
                self.getPaths(word)
        return
            
    
    def correctIndexes(self):
        for anno in self.annotationList:
            offset = 0
            anno.words = sorted(anno.words, key=lambda x : x.index)
            for word in anno.words:
                word.index = word.index - offset
                offset = offset + word.word.count("-")
         
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
    
    def _makeAnnnotationMLNCompatible(self, anno):
        for dep in anno.deps:
            dep.type = utils.toMLNCompatibleFormat(dep.type)
        
        for word in anno.words:
            if word.tag is not None:
                word.tag = utils.toMLNCompatibleFormat(word.tag)
            if word.context is not None:
                word.context = utils.toMLNCompatibleFormat(word.context)