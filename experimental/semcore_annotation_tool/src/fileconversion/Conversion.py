'''
Created on Jul 14, 2012

@author: meyer
'''
import json
import preprocessing
import utils
from reportlab.lib.validators import isInstanceOf
import IN

class abstractAnno:
    def __init__(self, inputString):
        self.originalString = ""
        self.convertedString = ""
        
class MturkToAnno(abstractAnno):
    '''
    classdocs
    '''
    
    def __str__(self):
        return self.convertedString
    
    def __init__(self, inputString):
        '''
        Constructor
        '''
        self.originalString = inputString
        self.convertedString = self.convertMturkToAnno(inputString)
        
    def convertMturkToAnno(self, inputString):
        """Convert a String containing Json data in the Mturk format to the annotationTool specific json"""
        jsonData = json.loads(inputString[0])
        
        result = dict()
        for entry in jsonData:
            if not entry['context'] in result:
                result[entry['context']] = dict()
                result[entry['context']]['textName'] = entry['context']
                result[entry['context']]['sentences'] = []
                result[entry['context']]['sentences'].append(dict())
                result[entry['context']]['sentences'][len(result[entry['context']]['sentences'])-1]['id'] = entry['context'].rsplit(';',1)[1]
                result[entry['context']]['sentences'][len(result[entry['context']]['sentences'])-1]['sentence'] = entry['sentence']
                result[entry['context']]['sentences'][len(result[entry['context']]['sentences'])-1]['words'] = []
            result[entry['context'  ]]['sentences'][len(result[entry['context']]['sentences'])-1]['words'].append(dict())
            currentWord = result[entry['context']]['sentences'][len(result[entry['context']]['sentences'])-1]['words'][len(result[entry['context']]['sentences'][len(result[entry['context']]['sentences'])-1]['words'])-1]
            currentWord['index'] = entry['index']
            currentWord['senses'] = []
            #Go through the sense
            for s in entry['results']:
                currentWord['senses'].append({'votes':s['votes'], 'sense':s['sense']})
                currentWord['tag'] = entry['tag']
                currentWord['word'] = entry['word']
                if 'entity' in entry:
                    currentWord['entity'] = entry['entity']
        return json.dumps(result.values())

class JsonToAnno(abstractAnno):
    
    def __str__(self):
        return self.convertedString
    
    def __init__(self, inputString):
        self.originalString = inputString
        self.convertedString = json.dumps(inputString)
        

class EhowToAnno(abstractAnno):
    
    def __str__(self):
        return self.convertedString
    
    def __init__(self, inputString):
        
        self.originalString = inputString
        self.convertedString = self.convertEhowToAnno(inputString)
        
    def convertEhowToAnno(self,inputString):
        #data = inputString.split('\n')
        data = inputString
        result = dict()
        result['textName'] = data[0]
        #data.pop(0)
        result['sentences'] = []
        for idx, l in enumerate(data):  
            result['sentences'].append(dict())
            result['sentences'][len(result['sentences'])-1]['id'] = idx
            result['sentences'][len(result['sentences'])-1]['sentence'] = l
        tmp = []
        tmp.append(result)
        return json.dumps(tmp)
    
class TextToAnno(abstractAnno):
    """Convert a free text to the Annotation format."""
    
    def __str__(self):
        return self.convertedString
    
    def __init__(self, input_string):
        self.originalString = input_string
        self.convertedString = self.text_to_anno(input_string)
    
    def text_to_anno(self, input_string):
        """Convert the input into anno format."""
        
        if not isinstance(input_string, str):
            raise(TypeError)
        if input_string is None or input_string=="":
            raise(ValueError)
        
        data = input_string.split('\n')
        result = dict()
        result['textName'] = data[0]
        result['sentences'] = []
        for idx, l in enumerate(data):  
            result['sentences'].append(dict())
            result['sentences'][len(result['sentences'])-1]['id'] = idx
            result['sentences'][len(result['sentences'])-1]['sentence'] = l
        res = []
        res.append(result)
        return json.dumps(res)
        
    
if __name__ == "__main__":
    """Execute the file conversion with file as input parameter."""
    import sys
    
    #inputString = sys.argv[1]
    #inputString = '[{"index":3,"results":[{"votes":5,"sense":"SID-07848338-N"}],"tag":"NN","context":"ADD;line 0","sentence":"Add sugar and butter to mixing bowl and beat well.","word":"butter"},{"index":5,"results":[{"votes":4,"sense":"SID-03775546-N"},{"votes":1,"sense":"OTHERTYPE"}],"tag":"NN","context":"ADD;line 0","sentence":"Add sugar and butter to mixing bowl and beat well.","word":"mixing bowl"},{"index":8,"results":[{"votes":5,"sense":"SID-01418179-V"}],"tag":"VB","context":"ADD;line 0","sentence":"Add sugar and butter to mixing bowl and beat well.","word":"beat"},{"index":0,"results":[{"votes":1,"sense":"NOMATCH"},{"votes":4,"sense":"SID-00182406-V"}],"tag":"VB","context":"ADD;line 0","sentence":"Add sugar and butter to mixing bowl and beat well.","word":"Add"},{"index":9,"results":[{"votes":3,"sense":"SID-00012779-R"},{"votes":2,"sense":"SID-00011093-R"}],"tag":"RB","context":"ADD;line 0","sentence":"Add sugar and butter to mixing bowl and beat well.","word":"well"},{"index":1,"results":[{"votes":5,"sense":"SID-07859284-N"}],"tag":"NN","context":"ADD;line 0","sentence":"Add sugar and butter to mixing bowl and beat well.","word":"sugar"},{"index":2,"results":[{"votes":3,"sense":"SID-13619168-N"},{"votes":2,"sense":"SID-13766733-N"}],"tag":"NN","context":"ADD;line 46","sentence":"Add 3\/4 cup sugar and beat until stiff peaks form.","word":"cup"},{"index":0,"results":[{"votes":5,"sense":"SID-00182406-V"}],"tag":"VB","context":"ADD;line 46","sentence":"Add 3\/4 cup sugar and beat until stiff peaks form.","word":"Add"},{"index":7,"results":[{"votes":1,"sense":"NOMATCH"},{"votes":1,"sense":"SID-01023706-A"},{"votes":3,"sense":"SID-01525659-A"}],"tag":"JJ","context":"ADD;line 46","sentence":"Add 3\/4 cup sugar and beat until stiff peaks form.","word":"stiff"},{"index":3,"results":[{"votes":5,"sense":"SID-07859284-N"}],"tag":"NN","context":"ADD;line 46","sentence":"Add 3\/4 cup sugar and beat until stiff peaks form.","word":"sugar"},{"index":9,"results":[{"votes":1,"sense":"NOMATCH"},{"votes":1,"sense":"SID-00142191-V"},{"votes":3,"sense":"SID-00144850-V"}],"tag":"VBP","context":"ADD;line 46","sentence":"Add 3\/4 cup sugar and beat until stiff peaks form.","word":"form"},{"index":8,"results":[{"votes":1,"sense":"SID-08617963-N"},{"votes":2,"sense":"NOMATCH"},{"votes":2,"sense":"SID-13902482-N"}],"tag":"NNS","context":"ADD;line 46","sentence":"Add 3\/4 cup sugar and beat until stiff peaks form.","word":"peaks"},{"index":5,"results":[{"votes":5,"sense":"SID-01418179-V"}],"tag":"VB","context":"ADD;line 46","sentence":"Add 3\/4 cup sugar and beat until stiff peaks form.","word":"beat"}]'
    inputString = "Heat oven to 350?F/180?C.\nPrepare and bake brownie mix as directed on package, using water, oil and eggs, in an 13 by 9 inch rectangular pan.\nCool completely In a chilled medium bowl, beat whipping cream with electric mixer on high speed until stiff.\nGently stir in marshmallows and cherries.\nSpread evenly over brownies.\nDrizzle chocolate over top.\nSwirl chocolate through whipped cream mixture with knife if desired Cover and refrigerate about one hour or until its chilled.\nServe."
    #conv = MturkToAnno(inputString)
    conv = EhowToAnno(inputString, "headline")
    print conv