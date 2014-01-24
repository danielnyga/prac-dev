'''
Created on Sep 17, 2012

@author: meyer
'''
import csv
import re
import os

class genderFileGenerator(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        pass
    
    def _mergeFiles(self, fileList):
        res = dict()
        number = False
        inserted = False
        filestructure = {'F':1,'M':2,'N':3,'P':4}
        for f in fileList:
            with open(f) as csvfile:
                wordreader = csv.reader(csvfile, delimiter='\t')
                for currentrow in wordreader:
                    if not currentrow[0] in res:
                        res[currentrow[0]] = self._createElementDictionaries(currentrow[1:len(currentrow)])
                    else:
                        res[currentrow[0]] = self._createElementDictionaries(currentrow[1:len(currentrow)], res[currentrow[0]])
                                        
        return res
    
    def _createGenderDictionary(self, mergedFileDict):
        res = dict()
        for key, value in mergedFileDict.iteritems():
            res[key] = self._getMajorityGender(value)
        return res
    
    def _getMajorityGender(self, valueDict):
        maxVal = 0
        second = None
        res = dict()
        res['gender'] = ''
        res['confidence'] = 0
        
        for key, value in valueDict.iteritems():
             if value > maxVal:
                 second = maxVal
                 maxVal = value
                 res['gender'] = key
        res['confidence'] = self._getConfidenceLevel(maxVal, second)
        
        return res

    def _getConfidenceLevel(self, maxVal, second):
        """Compute the confidence Level as a margin: margin = (f2-f1)f2."""
        
        res = float(((maxVal-second)+1))/float(second+1)
        return res

    def _createElementDictionaries(self, row, oldrow={'F':0,'M':0,'N':0,'P':0}):
        """Create a list of dictionaries of the form {'N':5, 'F':6} from input of the form ['N',5,'F',6]."""
        
        res = dict()
        res = {'F':0,'M':0,'N':0,'P':0}
        for idx, col in enumerate(row):
            if re.match('[A-Z]', str(col)) is not None:
                res[col] = int(oldrow[col]) + int(row[idx+1])
        return res
    
    def createConfidenceDictionary(self, fileList):
        """Create a dictionary with the NP and the confidence level."""
        
        dic = self._mergeFiles(fileList)
        res = self._createGenderDictionary(dic)
        return res
    
    def confidenceDictionaryToFile(self, confiDict, file):
        """Create a csv File from a Confidence Dictionary."""
        
        f = file
        with open(f, 'wb') as outPutFile:
            outputwrite = csv.writer(outPutFile,delimiter='\t')
            for key, value in confiDict.iteritems():
                outputwrite.writerow([key, value['gender'], value['confidence']])
        
        return True 
    
    
if __name__ == '__main__':
    g = genderFileGenerator()
    this_dir, this_filename = os.path.split(__file__)
    base_dir = os.path.join(this_dir, '..','..','annotation', 'div','ngrams','Gender')
    basefiles = ['Name-ConjPossessive.txt', 'Name-Predicate.txt', 
             'Name-VerbPossessive.txt', 'Noun-ConjPossessive.txt', 
             'Noun-VerbNominative.txt', 'Noun-VerbReflexive.txt',
             'Name-Designate.txt', 'Name-VerbNominative.txt',
             'Name-VerbReflexive.txt', 'Noun-Predicate.txt', 'Noun-VerbPossessive.txt']
    
    files = [os.path.join(base_dir, x) for x in basefiles]
    
    row = ['N',5,'F',6]
    #print g._createElementDictionaries(row)
    confi =  g.createConfidenceDictionary(files)
    g.confidenceDictionaryToFile(confi, 'confidenceGender.csv')
    