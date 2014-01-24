'''
Created on Jul 18, 2012

@author: meyer
'''
import re
import json
import logging
import operator
import sys
import os

class indexCorrecter:
    '''
    Class that corrects the indexes for mturk files(to start with 1), 
    specifically for two word nouns.
    '''
    
    def __init__(self,file):
        '''
        Constructor
        '''
        self.fileName = file
        self.file = self.correctIndexInFile(file) 
    
    def saveDataToFiles(self):
        filename = self.fileName.rsplit('/',1)
        newf = open(filename[0] + "/processed_" + filename[1] , 'w')
        newf.write(self.file)
        newf.close()
            
    def correctIndexInFile(self,f):
        content = self.loadFile(f)
        #Get a list of lines
        content[0] = content[0].strip()
        content[0] = content[0].rstrip(']')
        content[0] = content[0].lstrip('[')
        lines = re.sub(',{"index','\n{"index', content[0]).splitlines()

        return self.doIndexCorrection(lines)
        
    def doIndexCorrection(self,lines):
        jsonData = [json.loads(x) for x in lines]
        #Sort by sentece
        res = dict()
        sortedList = sorted(jsonData,key=lambda sentence : sentence['sentence'])
        
        for entry in sortedList:
            res[entry['sentence']] = filter(lambda x : x['sentence'] == entry['sentence'] , sortedList)
        sortedList = []
        for entry in res.itervalues():
            sortedList.append(sorted(entry, key=operator.itemgetter('index')))
        #index starts at 1! => offset init with 1
        resultList = []
        for entry in sortedList:
            offset = 1
            correctWord = []
            logging.info('\ncorrecting indexes for sentence: "%s"', entry[0]['sentence'])
            for en in entry:
                en['index'] = int(en['index']) + offset
                logging.info('correcting index for word: "%s"', en['word'])
                if len(en['word'].split(' ')) > 1:
                    offset = offset - len(en['word'].split(' '))
                    correctWord.append(en['word'])
                    en['word'] = re.sub(' ', '-', en['word'])
                    #correct the sentence for all words in the sentence at the end
            for en in entry:
                for w in correctWord:
                    en['sentence'] = re.sub(w, re.sub(' ', '-', w), en['sentence'])
                resultList.append(en)
        return json.dumps(resultList)
    
    def loadFile(self,f):
        openFile = open(f, 'r')
        result = openFile.readlines()
        openFile.close()
        return result
        
if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)
    data = sys.stdin.readlines()
    for d in data:
        d = d.rstrip()
        ic = indexCorrecter(d)
        ic.saveDataToFiles()