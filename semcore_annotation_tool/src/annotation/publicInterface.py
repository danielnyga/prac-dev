import fileconversion
import TextManager
import jpype
import os
from StanfordParser import *
import pickle
import logging
import csv
from statistics.database import *
import re

class annotationInterface:
    
    """The main interface for all modules trying to 
    access the annotation functionality.
    
    Public methods:
    __init__ --- Public constructor
    loadTexts --- Load another set of texts and add to existing list of texts.
    
    Public fields:
    texts --- list of all loaded texts.
    
    """
    
    def __init__(self, texts=None, fileList=None, kr = None):
        #TODO: Add configs for everything either in the header or config file
        """Public constructor.
        
        Key arguments:
        texts --- List of Strings
        files --- List of filenames. Either *.json, *.plain, *.ehow or *.mturk  
        
        """
        classpath = [os.path.join('..', 
                                  '..', 
                                  '3rdparty', 'stanford-parser-2012-02-03', 
                                  'stanford-parser.jar')]
        print jpype.getDefaultJVMPath()
        jpype.startJVM(jpype.getDefaultJVMPath(), 
                       '-ea', 
                       '-Djava.class.path=%s' % (':'.join(classpath)))
        logging.info('Starting Parser')
        self.parser = Parser(os.path.join('..'
                                          ,'..'
                                          ,'3rdparty'
                                          ,'stanford-parser-2012-02-03',
                                          'grammar',
                                          'englishPCFG.ser.gz'))
        self.texts = dict()
        self.create_rw_objects = False 
        self.genderDict = self._getGenderDict()
        self.kr = kr
        t = []
        if not fileList is None:
            t = self.loadFiles(texts, fileList)
        
        if texts is not None:
            t = t + self.loadFiles(texts, None)
            
        for text in t:
            textEnt = TextManager.TextManager(text.convertedString, 
                                              self.parser, 
                                              genderDict = self.genderDict, 
                                              kr = self.kr).textEntities
            for te in textEnt:
                loc = len(self.texts.keys())
                self.texts[loc] = te
            
    def addFiles(self, fileList):
        if not fileList is None:
            t = self.loadFiles(None,fileList)
            for text in t:
                textEnt = TextManager.TextManager(text.convertedString, 
                                                  self.parser, 
                                                  self.genderDict, 
                                                  kr = self.kr).textEntities
                for te in textEnt:
                    loc = len(self.texts.keys())
                    self.texts[loc] = te
                    
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
    
    def getMLN(self):
        realWorld=self.create_rw_objects
        result = []
        for t in self.texts.itervalues():
            result.append(t.getMLN(realWord=realWorld))
        return result
    
    def getMLNbyTextID(self, id):
        realWorld = self.create_rw_objects
        res = self.texts[id].getMLN(realWorld = realWorld)
        return res
    
    def saveTextsToSQLite(self, filename):
        """
        Save the texts to a SQLite database.
        """
        
        #Create the database schema
        i = Initialization()
        conn = i.get_db('statistics.db')
        i.create_statistics_schema(conn)
        
        for t in self.texts.values():
            t.saveTextToSQLite(filename)
    
    def saveToMLNDatabase(self,folder):
        """
        Save each text into one MLN file that is used as
        a database for the MLN.
        
        """
        realWorld=self.create_rw_objects
        for key, val in self.texts.iteritems():
            f = open(os.path.join(str(folder), str(key) + '.mln'),'w')
            res = val.getMLN(realWorld=realWorld)
            f.write(res)
            f.close()
        
    def saveSelectedtoMLN(self, folder, text, 
                          saveToFile = None):
        realWorld=self.create_rw_objects
        if saveToFile is None:
            f = open(os.path.join(str(folder), str(text) + '.mln'),'w')
        else:
            f = open(saveToFile, 'w')
        res = self.texts[int(text)].getMLN(realWorld=realWorld)
        f.write(res)
        f.close()
    
    def saveData(self, filename):
        for t in self.texts.itervalues():
            t.text.kr = None
            t.text.parser = None
            t.text.genderDict = None
        try:
            f = open(filename,'w')
            pickle.dump(self.texts, f, pickle.HIGHEST_PROTOCOL)
            f.close()
        except Exception as e:
            print "Error saving to file.Msg.: {msg}".format(msg=e)
    
    #Load a pickled python data file and create a list of annotations
    def loadProgressFromFile(self, filename):
        try:
            f =open(filename,'r')
            self.texts = pickle.load(f)
        except Exception:
            return False
        for t in self.texts.itervalues():
            t.text.kr = self.kr
            t.text.genderDict = self.genderDict
            t.text.parser = self.parser
        return True
        
    
    def loadFiles(self, texts, files):
        
        """Load another set of texts and add to existing list of texts.
        
        Key arguments:
        files --- List of filenames. Either *.plain, *.ehow or *.mturk
        
        """
        
        cm = fileconversion.ConversionManager()
        result = []
        if not files is None:
            result = result + cm.convertFiles(files)
        if not texts is None:
            result = result + cm.convertFiles(texts)
            
        return result
        

if __name__=="__main__":
    import logging
    from statistics.database import *
    logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)
    fileList = []
    fileList.append(
        "/usr/stud/meyer/Source/semcore_annotation_tool/src/testMLN/test.ehow")
    
    m = annotationInterface(texts = None, fileList = fileList)
    i = Initialization()
    conn = i.get_db('statistics.db')
    i.create_statistics_schema(conn)
    #m.saveTextToSQLite()