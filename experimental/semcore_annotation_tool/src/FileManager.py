#Class that handles all the file operations
import re
import utils
try:
    import cPickle as pickle
except:
    import pickle
#Path where the annotated files will be written to relative to starting directory
savepath = "../../data/annotated_sentences/"

#The path of the role file
roles = "roles/roles.db"

#File extension of the saved files
fileExtension = ".db"

class FileManager:
    
    def __init__(self):
        self.saveLocation = savepath
        
    def saveProgressToFile(self,annotationList, filename):
        try:
            f = open('./annotationProgress.pkl','w')
            pickle.dump(annotationList, f, pickle.HIGHEST_PROTOCOL)
            f.close()
        except:
            print "Error saving to file."
            
    #Load a pickled python data file and create a list of annotations
    def loadProgressFromFile(self, filename):
        f = open(filename,'r')
        return pickle.load(f)
    
    def convertTagToWordnetPOS(self,tag):
        #DRemove first statement for a different abstraction
        return tag
        if tag in ['NN', 'NNP', 'NNPS', 'NNS']:
            return 'n'
        if tag in ['VB', 'VBD', 'VBG', 'VBN', 'VBP', 'VBZ']:
            return 'v'
        if tag in ['RB', 'RBR', 'RBS']:
            return 'r'
        if tag in ['JJ', 'JJR', 'JJS']:
            return 'a'
        return None
        
    #Save the output of the textout to a file
    def saveSentence(self, sentence, data):
        filename = self.makeFileNameFromSentence(sentence)
        self.writeToFile(self.saveLocation + filename + fileExtension, data)
        
    #Create a fileName from the sentence
    def makeFileNameFromSentence(self, sentence):
        #re.sub("[\W]","",e)
        result = '_'.join(self.eraseSpecialandLower(e) for e in sentence.split())
        return result   
    
    #Write Textdata to a file with a given filename
    def writeToFile(self, filename, data):
        FILE = open(filename, "w")
        FILE.writelines(data)
        FILE.close()
        
    #Delete all special characeters of a string and change to lower case
    def eraseSpecialandLower(self, exp):
        exp = re.sub("[\W]","",exp)
        exp = exp.lower()
        return exp        
    
    #Load the role file and return a list of the roles
    def loadRoles(self):
        f = open(roles, 'r')
        result = f.readlines()
        f.close()
        return result

    def exportAllAnnoToMLN(self,annotations):
        for anno in annotations:
            self.exportAnnoToMLN(anno)
    
    #Export one entire annotation to a MLN
    def exportAnnoToMLN(self, anno):
        #Write a headline
        result = "" 
        for word in anno.words:
            result += "// " + word.word + "\n"
            result += self.getPOSMLNString(word, anno.sentenceID)
            result += self.getSenseMLNString(word, anno.sentenceID)
            result += self.getRolesMLNString(word, anno.sentenceID)
            result += self.getPathMLNString(word)
            result += "\n"
        result += self.getDepsMLNString(anno.deps, anno.sentenceID)
        self.saveSentence(anno.sentence, result)
        print result
    
    def getPOSMLNString(self,word, sentenceID):
        if word.word=="Root":
            return ""
        return 'hasPOS(%s,%s,%s)\n'%(((utils.toMLNCompatibleFormat(word.word)) + str(sentenceID) + str(word.index)),(self.convertTagToWordnetPOS(word.tag)),'S'+str(sentenceID))
    
    #Return the output string for all the path relations of a word
    def getPathMLNString(self,word):
        result = "\n//Taxonomy:\n\n"
        if not word.result is None:
            if word.getSenseList()[word.selectedSense]['sense'] == '':
                return result
            elif word.getSenseList()[word.selectedSense]['sense'] == "NOMATCH":
                return result
            elif word.getSenseList()[word.selectedSense]['sense'] == "OTHERTYPE":
                return result
            for deg in word.getSenseList()[word.selectedSense]['path'][0]:
                result += 'isa(%s, %s)\n'%(str(word.getSenseList()[word.selectedSense]['sense']),utils.toMLNCompatibleFormat(deg.name))
        return result
        
    #Return output string for one wordSense
    def getSenseMLNString(self, word, sentenceID):
        if word.word=="Root":
            return ""
        if word.result is None:
            return 'hasSense(%s,%s,%s)\n'%(((utils.toMLNCompatibleFormat(word.word)) + str(sentenceID) + str(word.index)),"NullSense",('S'+str(sentenceID)))
        return 'hasSense(%s,%s,%s)\n'%(((utils.toMLNCompatibleFormat(word.word)) + str(sentenceID) + str(word.index)),(word.getSenseList()[word.selectedSense]['sense']),'S'+str(sentenceID))
    
    #Return the MLN output string for all dependencies of one annotation
    def getDepsMLNString(self, deps, sentenceID):
        result = "//Dependencies from the Stanford Parser\n\n"
        for dep in deps:
            typ = utils.toMLNCompatibleFormat(dep.type)
            origin = utils.toMLNCompatibleFormat(dep.origin.word) + str(sentenceID) + str(dep.origin.index)
            destiny = utils.toMLNCompatibleFormat(dep.destiny.word) + str(sentenceID) + str(dep.destiny.index)
            sentence = "S" + str(sentenceID)
            result += '%s(%s,%s,%s)\n'%(typ, origin, destiny,sentence)      
        return result 
    
    
    #Return the MLN output string for the roles of one word
    def getRolesMLNString(self,word, sentenceID):
        result = ""
        if word.word == "Root":
            return ""
        if word.tag is None:
            return 'hasRole(%s,%s,%s)\n'%(((utils.toMLNCompatibleFormat(word.word)) + str(sentenceID) + str(word.index)),"NULL",('S'+str(sentenceID)))
        for role in word.roles:
            result += 'hasRole(%s,%s,%s)\n'%(((utils.toMLNCompatibleFormat(word.word)) + str(sentenceID) + str(word.index)),(role),'S'+str(sentenceID))
        return result
    