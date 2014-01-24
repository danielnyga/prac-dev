import sys
from compiler.ast import Sub
sys.path.append("/usr/stud/meyer/Source/edu.tum.cs.prac/src")
import json
import Sentence

#import FileManager
import logging
import os


#Manages multiple texts
class TextManager:
    
    """Handle multiple texts given in the specified format by the annotationTool.
        
    public methods:
    __init__ --- Public constructor
    printMLN --- Return the MLN representation as String 
        
    """
    
    def __init__(self, texts, parser, genderDict=None, kr = None):
        
        """Public constructor
        
        Keyword arguments:
        texts -- a list of different texts in the specified format by the annotationTool
        
        """

        logging.info('Starting to load new list of texts')
        self.kr = kr
        self.parser = parser
        self.textEntities = self._textsToAnnotations(texts, parser, genderDict)
        logging.info('New texts loading complete')

    def _textsToAnnotations(self, texts, parser, genderDict=None):
        """Parse a list of JSON texts to an Annotation."""
        logging.info('starting to parse JSON')
        parsedTexts = []
        loadedText = json.loads(texts)
        i =0
        for pt in loadedText:
            #try:
            parsedTexts.append(TextEntity(i, self._textToAnnotations(pt, parser, genderDict, self.kr)))
            i+=1
            #except Exception as e:
                    #logging.error('parsing the JSON file failed! Msg.:' + str(e))
                    #exit()
        logging.info('JSON file parsed')
        return parsedTexts
        
    def _textToAnnotations(self, text, parser, genderDict=None, kr = None):
        """
        Parse a JSON text to an Annotation.
        """
        
        logging.info('create new sentenceManager')
        return Sentence.SentenceManager(parser, text['sentences'],
                                        genderDict=genderDict,
                                        kr=kr,
                                        text_name = text['textName'])
    
    def getMLN(self, realWorld=False):
        """Print the MLNs for all texts"""
        res = []
        for te in self.textEntities:
            #TODO: implement grounding of entities into the real world here!
            res.append(te.text.getMLN(te.id, realWorld=realWorld))
            #result += self.MLNGroundings
            #result += te.MLNView
        return res
    
    def saveTextToSQLite(self, database_name):
        """
        Save data to a sqlite database for all texts.
        """
        
        for t in self.textEntities:
            t.saveTextToSQLite(t.id, database_name)

class TextEntity:
    def __init__(self, id, t):
        self.id = id
        self.text = t
    
    def __str__(self):
        return self.id
    
    def getMLN(self, realWorld=False):
        return self.text.getMLN(self.id, realWorld=realWorld)
    
    def saveTextToSQLite(self, database_name):
        self.text.saveTextToSQLite(database_name)

if __name__ == "__main__":
    import jpype
    from StanfordParser import *
    import subprocess
    """Run a standalone version of the file(Needs its dependencies!.. of course)."""
    logging.basicConfig(format='%(asctime)s %(message)s', level=logging.INFO)
    classpath = [os.path.join('..', '..', '3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar')]
    print jpype.getDefaultJVMPath()
    jpype.startJVM(jpype.getDefaultJVMPath(), '-ea', '-Djava.class.path=%s' % (':'.join(classpath)))
    parser = Parser(os.path.join('..','..','3rdparty','stanford-parser-2012-02-03','grammar','englishPCFG.ser.gz'))
    test = '[{"textName":"Making pancakes", "sentences":[{"id":"0","sentence":' \
            '"Add sugar and butter to mixing-bowl and beat well.","words":[' \
            '{"index":4,"senses":[{"votes":5,"sense":"SID-07848338-N"}],'\
                        '"entity":"w002",'\
                        '"roles":[{"role":"object"}],'\
                        '"tag":"NN","context":"ADD;line 0",' \
                        '"sentence":"Add sugar and butter to mixing-bowl and beat well."' \
                        ',"word":"butter"},'\
            '{"index":6,'\
                        '"senses":[' \
                            '{"votes":4,"sense":"SID-03775546-N"},' \
                            '{"votes":1,"sense":"OTHERTYPE"}],' \
                        '"entity":"w001",'\
                        '"roles":[{"role":"destinationFrameNetID"},{"role":"locationFrameNetID"}],'\
                        '"tag":"NN","context":"ADD;line 0",' \
                        '"sentence":"Add sugar and butter to mixing-bowl and beat well.",' \
                        '"word":"mixing-bowl"},'\
            '{"index":8,"senses":[{"votes":5,"sense":"SID-01418179-V"}],' \
                        '"roles":[{"role":"actionVerb"}],'\
                        '"tag":"VB","context":"ADD;line 0",' \
                        '"sentence":"Add sugar and butter to mixing-bowl and beat well.",' \
                        '"word":"beat"},'\
            '{"index":1,"senses":' \
                        '[{"votes":1,"sense":"NOMATCH"},'\
                        '{"votes":4,"sense":"SID-00182406-V"}],' \
                        '"roles":[{"role":"actionVerb"}],'\
                        '"tag":"VB","context":"ADD;line 0","sentence":' \
                        '"Add sugar and butter to mixing-bowl and beat well.",' \
                        '"word":"Add"},'\
            '{"index":9,"senses":[' \
                        '{"votes":3,"sense":"SID-00012779-R"},'\
                        '{"votes":2,"sense":"SID-00011093-R"}],' \
                        '"roles":[{"role":"postitiveModifier"}],'\
                        '"tag":"RB","context":"ADD;line 0",' \
                        '"sentence":"Add sugar and butter to mixing-bowl and beat well.",' \
                        '"word":"well"},'\
            '{"index":2,"senses":[{"votes":5,"sense":"SID-07859284-N"}],' \
                        '"entity":"w003",'\
                        '"roles":[{"role":"object"}],'\
                        '"tag":"NN","context":"ADD;line 0",' \
                        '"sentence":"Add sugar and butter to mixing-bowl and beat well.",' \
                        '"word":"sugar"}]}'\
            ',{"id":"1","sentence":"Make yummy cookies!"}]}]'
    tm = TextManager(test, parser)
    print tm.printMLN()[0]