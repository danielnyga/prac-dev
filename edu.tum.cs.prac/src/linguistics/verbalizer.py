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

# File definition: 
#Main class for PRACverbalization, 
#resolves the syntactic context dependencies defined in such file, 
#and substitutes the variables in the DRS and proceeds in processing 
#it via the DRS>ACE prolog module. 

import re
import yaml

import sys
import os
import subprocess
from os.path import abspath, dirname, join

from actioncore import *
from linguistics import *

from utils import StopWatch, red, bold
from utils import bash

import operator
from linguistics.verbshell import VerbShell

class queryCase(object):
    def __init__(self, role, queryType, param, typ):
        self.role = role
        self.queryType = queryType
        self.param = []
        if param is not None:
            if (typ == "array"):
                for pm in param:
                    temp = re.findall(r'[a-zA-Z_]+', pm)
                    self.param.append(temp[0])
            else: 
                    tmp = re.findall(r'[a-zA-Z_]+', param)
                    self.param.append(tmp[0])

    def printCase(self):
        pass
        print "  - Ask for missing role " + "\"" + bash.BOLD + self.role + bash.END + "\"" + "," + " case " + "\"" + bash.BOLD + self.queryType + bash.END + "\"  " +  bash.BOLD + '%s' % ', '.join(map(str, self.param)) + bash.END

class DRStoNL(object):
    def __init__(self,drs):
        self.drs = drs
        self.nl = None

    def create(self):
        ex_path = abspath(join(dirname(__file__), '../linguistics/tools/./drsverb'))
        args = ex_path +" " + "\"" + self.drs + "\""
        l = subprocess.check_output(args, stderr=subprocess.STDOUT, shell=True)
        print bash.RED + l + bash.END

class PRACVerbalizer(PRACReasoner):

    def __init__(self):
        super(PRACVerbalizer, self).__init__('verbalizer')
        self.DRSdep = None
        self.drs = []
        self.cases = []
        self.nlitags = NLISentence()

    
    @PRACPIPE
    def run(self):

        self._instanciateCtrlTemplate()
        
        self._understandTargetCase()
        if (self.cases is not None):
            self._pull_ctrl_syn_features() # or fallback to last_syn_features to be implemented
            self._pullDrsCases()
            print bash.BOLD + "Verbalization: " + bash.END
            for query in self.drs:
                reply = DRStoNL(query)
                reply.create()
            print 
            print
            print bash.BOLD + "Integrating Reply Information" + bash.END
                        
#VerbShell().cmdloop()

#    def _verbManipulation(self, verb_inf_form, man_type):
#        --passive
#        --ing form

    def _instanciateCtrlTemplate(self):
    
        self.synParser = self.pracinference.synParser
        ##Instantiating ctrl template
        resultDB = self.pracinference.databases['core']
        actioncore = self.pracinference.actioncore
        action_cores_path = os.path.join('models', 'actioncores.yaml')
        dataMap = yaml.load_all(open(action_cores_path))
        for elem in dataMap:
             if (elem['action_core'].lower() == "flipping"):
                 self.pracinference.ctrlStatement = elem['controlled_template'].lower()
        for var in resultDB.query('action_role(?w, ?r) ^ !(?r=NULL) ^ !(?r=ActionVerb)'):
            m = re.findall(r'[a-zA-Z_]+', var['?w'])
            #print "role: " + var['?r'] + " and value: " + var['?w'] + " trimmed: " + m[0]
            self.pracinference.ctrlStatement = self.pracinference.ctrlStatement.replace(var['?r'].lower(),m[0].lower()) 
            #print self.pracinference.ctrlStatement    

    def _pullDrsCases(self):
        f = open(abspath(join(dirname(__file__), '../linguistics/DRStemplates.yaml')))
        dataMap = yaml.load(f)
        f.close()
        for query in self.cases:
                 dep = dataMap["query_cases"][query.queryType]["dependencies"]
                 self.drs.append(dataMap["query_cases"][query.queryType]["drs"])
                 self.drs[-1] = self._solve_dependencies(dep, query.param, self.drs[-1])

    def searchCtrlTemplate(self,role):
        ctrl_deps = self.synParser.getDependencies(self.pracinference.ctrlStatement, True)

        self.ctrltags = NLISentence()
        cldeps = map(str, ctrl_deps)

        self.ctrltags.add(cldeps)

        for item in self.ctrltags.SYNlist:
                 if (item.left.lower() == role.lower() or item.right.lower() == role.lower()):
                     missingSynTag = item.syntype.lower()
                     if (missingSynTag != "det"):
                         self.cases.append(queryCase(role, missingSynTag, None, None)) 

    def _understandTargetCase(self):

        print bash.BOLD + 'Question case evaluation' + bash.END
        print bash.BOLD + '========================' + bash.END
        print

        features = self.pracinference.features
        feature = features.get('missingroles')
        
        global_threshold = 0.2
        relative_threshold = 0.1

        for role in feature.missingRoles:
            sortedList = sorted(self.pracinference.role_distribution[role], reverse=True)
            distribution = sorted([(str(l.params[1]),v) for l, v in self.pracinference.role_distribution[role].iteritems()], key=operator.itemgetter(1), reverse=True)
            if (distribution[0][1] < global_threshold):
                self.cases.append(queryCase(role, "impossibility", str(distribution[0][0]),"single"))
            elif (distribution[0][1] - distribution[1][1] < relative_threshold):
                param = []
                param.append(distribution[0][0])
                param.append(distribution[1][0])
                self.cases.append(queryCase(role, "two-choices", param, "array"))
            self.searchCtrlTemplate(role)             

        if (self.cases is not None):
            print bash.BOLD + 'To be asked:' + bash.END
            for cs in self.cases:
                cs.printCase()
        else:
            print bash.BOLD + "Nothing to be asked." + bash.END 
        print  

    def _pull_ctrl_syn_features(self):
        ctrl_deps = self.synParser.getDependencies(self.pracinference.ctrlStatement, True)
        cldeps = map(str, ctrl_deps)
        self.nlitags.add(cldeps)        

    def _pull_last_syn_features(self):

        features = self.pracinference.features
        syntags = []
        syntags.append(features.get('syntax').getEvidence())        
        for elem in syntags:
                 self.nlitags.add(elem)        

    def _solve_dependencies(self, dep, param, currentdrs):
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
            if (self.DRSdep.look_side[i].lower() == "ext"):
                self.DRSdep.look_subst[i] = param[i]

	    for j in self.nlitags.SYNlist:
	        if (j.syntype.lower() == self.DRSdep.look_var[i].lower()):
	            if (self.DRSdep.look_side[i].lower() == "left"):
		        self.DRSdep.look_subst[i] = j.left.lower()
    
	            elif (self.DRSdep.look_side[i].lower() == "right"): 
	                self.DRSdep.look_subst[i] = j.right.lower()    
                            
        for i in range(len(self.DRSdep.look_subst)):
            currentdrs = currentdrs.replace(eliminate[i],self.DRSdep.look_subst[i])
        return currentdrs



