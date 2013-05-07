# PROBABILISTIC ROBOT ACTION CORES - REPLY INTEGRATION FUNCTIONS
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
#Main class for PRACIntegrator, class for melding pracinstances

from actioncore import *
from utils import red, bold
from utils import bash

from pracmln.PRACDatabase import PRACDatabase

class PRACIntegrator(PRACReasoner):

    def __init__(self):
        super(PRACIntegrator, self).__init__('integrator')
        self.assignedRoles = {}

    @PRACPIPE
    def run(self):

        bfrdic = {}
        for val in self.pracinference.databases['core'].query('action_role(?w, ?r)'): #action_role(?w, ?r) ^ !(?r=NULL) ^ !(?r=ActionVerb)
              bfrdic.update({val['?r']: val['?w']})

        for key, value in bfrdic.items():
            present = False
            for chkKey, chkValue in self.assignedRoles.items():
                if (key == chkKey or key == 'NULL'):
                   present = True
            if (present == False):
                self.assignedRoles[key] = value
            
        print bash.BOLD + "Current Understanding: \n" + bash.END
        for key, value in self.assignedRoles.items():
            print bash.BOLD + '    ' + bash.OKGREEN + key + bash.END + ': ' + value
        print '\n'



