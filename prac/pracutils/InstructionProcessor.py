from os import path, listdir
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
import sys
from RolequeryHandler import RolequeryHandler

if __name__ == '__main__':
    args = sys.argv[1:]
    
    path_to_corpus = args[0]
    corpus = []
    
    if path.isdir(path_to_corpus):
        corpus = map(lambda x: path.abspath(path.join(path_to_corpus,x)),listdir(path_to_corpus))
    
    elif path.isfile(path_to_corpus):
        corpus.append(path.abspath(path_to_corpus))
        
    else:
        print "No valid corpus path"
    
    for text_file_path in corpus:    
        text_file = open(text_file_path, 'r')
        sentences = text_file.readlines()
        
        prac = PRAC()
        prac.wordnet = WordNet(concepts=None)
            
        #Parse text file name to annotate it in the mongo db
        inference = PRACInference(prac, ["{}.".format(path.basename(text_file_path))])
        while inference.next_module() != 'achieved_by' and inference.next_module() != 'plan_generation':
            modulename = inference.next_module()
            module = prac.getModuleByName(modulename)
            prac.run(inference, module)
        
        #There will be only one db
        db = inference.inference_steps[-1].output_dbs[0]
        
        roles_dict = RolequeryHandler.queryRolesAndSensesBasedOnActioncore(db)
        
        #Process all sentences
        plan_list = []
        for s in sentences:
            inference = PRACInference(prac, [s])
            while inference.next_module() != None :
                modulename = inference.next_module()
                module = prac.getModuleByName(modulename)
                prac.run(inference, module)
            step = inference.inference_steps[-1]
                
            if hasattr(step, 'executable_plans'):
                plan_list.extend(step.executable_plans)
                
        print plan_list            
