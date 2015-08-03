import os
import re
from prac.wordnet import WordNet

class ActionCoreDbCreator(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    ACTIONCORE_DB_PATH = os.path.join(PRAC_HOME, 'praccore', 'pracutils','ActioncoreDbCreator',"actioncore_dbs")
    
    def initSynsetKeyList(self):
        wordnet = WordNet(concepts=None)
        synset_key_list = []
        
        for filename in os.listdir(self.ACTIONCORE_DB_PATH):
            if filename.endswith("~"): continue
            file = open(os.path.join(self.ACTIONCORE_DB_PATH,filename),'r')
            content = file.read()
            regex_result = re.search('\s*//SYNSET_KEY\s*:\s*(\w+)',content)
            print filename
            if regex_result:
                synset_key_list.append(wordnet.synsets(str(regex_result.group(1)),"v"))
        
        return synset_key_list
    
    

if __name__ == '__main__':
    ac = ActionCoreDbCreator()
    list = ac.initSynsetKeyList()
    
    print list
            
            
    
    