# PROBABILISTIC ROBOT ACTION CORES - CLASS HIERARCHY FOR SYNTACTIC DEPENDENCIES
#
# (C) 2013 by Nicholas H. Kirk
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import re
import yaml


class HRDialog(object):
    '''
    class representing the Human-Robot Dialogs,
    possibly including history of sessions.
    To be called in the outer module.
    '''
    def __init__(self):
        self.Sentencelist = []
        
    def add(NLISentence):
        self.SentenceList.append(NLISentence)

class NLISentence(object):
    '''
    representation of an instruction or clarification
    in natural language, coming from the human.
    We encapsulate the syntactic dependencies coming
    from the Stanford Parser.
    '''
    
    def __init__(self):
        self.SYNlist = []
        
    def add(self, stringM):
        for i in stringM:
            self.SYNlist.append(SYNDependency(i))    

class SYNDependency(object):
    '''
    representation of a single syntactic dependency
    of the Stanford Parser. Contains 3 string members
    accessible for non-logical string substitution
    purposes in linguistics.
    '''        

    def __init__(self, string):
        
        m = re.findall(r'[a-zA-Z_]+', string)

        self.syntype = m[0]
        self.left    = m[1]
        self.right   = m[2]
        

class DRSdependency(object):
    def __init__(self, look_var, look_side, subst):
        self.look_var = look_var
        self.look_side = look_side
        self.look_subst = subst
