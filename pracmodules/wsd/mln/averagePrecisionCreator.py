import sys
from utils.eval import ConfusionMatrix
import os
import re

class WritableObject:
    def __init__(self):
        self.content = []
    def write(self, string):
        self.content.append(string)
        
if __name__ == '__main__':
    
    
    
    pattern = r'^\w{3}_\d{2}_\w{3}_\d{4}_\d{2}:\d{2}:\d{2}_K=\d+_TSC=\d+$'
    gen = (f for f in os.listdir('.') if re.search(pattern, f))
    
    print 'Start compare process...'
    
    writable = WritableObject()                   
    sys.stdout = writable
    for f in gen:
        ConfusionMatrix.compareConfusionMatrices(os.path.join(f,'FOL','conf_matrix.cm'),os.path.join(f,'FUZZY','conf_matrix.cm'))
    
    sys.stdout = sys.__stdout__    
    
    f = open('Compare_Results', 'w')
    for content in writable.content:
        f.write(content)
    f.close()
    print 'done'
    
    