'''
Created on Sep 6, 2012

@author: meyer
'''
import re
class helper:
    def __init__(self):
        pass
    
    @staticmethod
    def handleSpecialChars(word):
        tags = {'\$':'__DOLLAR__','\.':'__POINT__',',':'__COMMA__', '\/':'__SLASH__','#':'__SHARP__',':':'__DOUBLE__'}
        res = word
        for key in tags.keys():
            res = re.sub(key,tags[key], res)
        return res