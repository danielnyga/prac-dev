# PROBABILISTIC ROBOT ACTION CORES - VERBALIZER FUNCTIONS
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

# File definition: ##TODO: improve hierarchy/class division
# Still in an immature prototypical phase, for now comprises the functions
# that parse the 'DRStemplates.yaml' file, resolves the syntactic context 
# dependencies defined in such file, and substitutes the variables in the DRS
# and proceeds in processing it via the DRS>ACE prolog module. 


import re
import yaml
from linguistics import *
#from actioncore.inference import PRACInit

#import roslib
#roslib.load_manifest('json_prolog')
#import rospy
#import json_prolog
from pyswip import Prolog

class DRStoNL(object):
    def __init__(self,drs):
        self.drs = drs
        self.nl = None
#        nodeName = 'PRACVerbalizer'
#        rospy.init_node(nodeName, anonymous=True)
#        self.prolog = json_prolog.Prolog()
      
    def create(self):

        self.drs = "drs([],[=>(drs([a],[object(a,man,countable,na,eq,1)-1/2]),drs([b,c],[object(b,human,countable,na,eq,1)-1/5,predicate(c,be,a,b)-1/3]))])"
        prolog = Prolog()
        prolog.consult('/home/nick/Workspace/PRAC/mturk/edu.tum.cs.prac/src/linguistics/utils/drs_to_coreace.pl')
        bf = "bigdrs_to_coreace("+ self.drs +",Ace)"
        print bf
   
        #JSon approach not currently working
#        q = self.prolog.query('[drs_to_coreace].')
#        c = self.prolog.query("bigdrs_to_coreace("+ self.drs +",Ace)")
#        for s in c.solutions(): print s
#        q.finish()
#        c.finish()
         


#        for result in prolog.query("sendmore(X)"):
#        r = result["X"]
#            for i, letter in enumerate(letters):
#                print letter, "=", r[i]

#        for result in prolog.query(bf):
#            r = result['Ace']
#            print r

class PRACVerbalizer(object):
    
    def __init__(self, NLISentence):
#        super(PRACInit, self).__init__('verbalizer')
        self.nli = NLISentence
        self.DRSdep = None
        self.drs = None
    
    def run(self):
        
        #LOADING THE YAML-SERIALIZED DRS TEMPLATES
        #f = open('DRStemplates.yaml')
        f = open('/home/nick/Workspace/PRAC/mturk/edu.tum.cs.prac/src/linguistics/DRStemplates.yaml')
        dataMap = yaml.load(f)
        f.close()
        
        #ACCESSING MEMBERS ##TODO: loading of custom templates
        dep = dataMap["syntactic_predicates"]["prep_to"]["dependencies"]
        self.drs = dataMap["syntactic_predicates"]["prep_to"]["drs"]
        self._solve_dependencies(dep)
        bfr = DRStoNL(self.drs)
        bfr.create()
        
    def _solve_dependencies(self, dep):
        '''
        String parsing operations to retrieve vectors of 
        deps defined in YAML
        '''
        look = []
        subst = []
        look_var  = []
        look_side = []
        eliminate = []
        
        look, subst = dep.split(";",1)
        look = look.split(",")
        for i in look:
            v, s = i.split("-")
            look_var.append(v)
            look_side.append(s)
        subst = subst.split(",")
        for i in range(len(look_var)):
	    look_var[i] = look_var[i].strip()
	    look_side[i] = look_side[i].strip()
	    subst[i] = subst[i].strip()        
        #finished string manipulation
        eliminate = subst[:]        
        self.DRSdep = DRSdependency(look_var, look_side, subst)
        
        for i in range(len(self.DRSdep.look_var)):
	    for j in self.nli.SYNlist:
	        if (j.syntype.lower() == self.DRSdep.look_var[i].lower()):
	            if (self.DRSdep.look_side[i].lower() == "left"):
		        self.DRSdep.look_subst[i] = j.left
	            elif (self.DRSdep.look_side[i].lower() == "right"): 
	                self.DRSdep.look_subst[i] = j.right    
        
        for i in range(len(self.DRSdep.look_subst)):
            self.drs = self.drs.replace(eliminate[i],self.DRSdep.look_subst[i])


