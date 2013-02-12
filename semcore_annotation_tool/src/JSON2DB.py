'''
Created on Mar 1, 2012

@author: nyga
'''

import json
import re
from nltk.corpus import wordnet as wn
import utils
    
class AnnotatedWord:
    #For a given sentence get the majority votes and output
    def __init__(self,index,word,results,sentenceID=None):
        self.index = index
        self.word = word
        self.roles = []
        maxvotes = 0
        self.sense = None
        self.sentenceID = sentenceID
        #For the sense the one with the most votes
        for r in results:
            v = r['votes']
            if v > maxvotes:
                maxvotes = v
                self.sense = r['sense']
                
    def __str__(self):
        keyName = self.word + '-'+str(self.index+1)
        return 'hasSense(%s,%s,%s)'%(utils.toMLNCompatibleFormat(str(keyName)),self.wordNetToNLTK(self.sense),utils.toMLNCompatibleFormat(str(self.sentenceID)))
    
    #Uses NLTK to transform a Wordnet ID to a NLTK ID that is MLN conform
    def wordNetToNLTK(self,wordnetID):
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
        #Synset('time.n.01')
        tmpID = re.search('\'.+\'', nltkID)
        nltkID = tmpID.group().strip('\'')
        #Make MLN conform
        nltkID = utils.toMLNCompatibleFormat(nltkID)
        return nltkID

class JSON2DB:
    
    def __init__(self, file):
        jsonfile = open(file, 'r')
        self.jsondata = json.load(jsonfile)
        self.sentences = set([])
        for annotation in self.jsondata:
            for key in annotation.keys():
                if key == 'sentence':
                    #List of setences!
                    self.sentences.add(annotation[key])
        self.sentences = list(self.sentences)
        
    #Holt alle annotation fuer einen Satz
    def getAnnotations(self, i):
        if i >= len(self.sentences):
            return None
        s = self.sentences[i]
        annotations = []
        annot = [x for x in self.jsondata if x['sentence'] == s]
        for a in annot:
            #This line gets one sense
            annotations.append(AnnotatedWord(index=a['index'],word=a['word'],results=a['results'],sentenceID='s'+str(i)))
        return annotations
    
if __name__ == '__main__':
    db = JSON2DB('/home/meyer/Workspace/edu.cs.tum.mturk.test/data/live_2/ADD/HITS.results')
    for i,s in enumerate(db.sentences):
        ann = db.getAnnotations(i)
        for a in ann:
            print str(a)
