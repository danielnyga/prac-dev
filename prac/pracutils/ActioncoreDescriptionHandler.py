# Markov Logic Networks
#
# (C) 2015 by Sebastian Koralewski (seba@informatik.uni-bremen.de)
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
import os

class ActioncoreDescriptionHandler(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    actioncoreDescriptionFilePath = os.path.join(PRAC_HOME, 'models', 'actioncores.yaml')
    actioncoreDescription = {}
    
    @staticmethod
    def loadActioncoreDescription():
        ADHandler = ActioncoreDescriptionHandler
        actioncoreRawList =re.compile("\n\s*-+\s*\n").split(open(ADHandler.actioncoreDescriptionFilePath).read())
        actioncoreYamlList = map(yaml.load,actioncoreRawList)
        
        for e in actioncoreYamlList:
            ADHandler.actioncoreDescription[e['action_core']] = e
    @staticmethod
    def getRolesBasedOnActioncore(actioncore):
        ADHandler = ActioncoreDescriptionHandler
        if ADHandler.actioncoreDescription == {}:
            ADHandler.loadActioncoreDescription()
        return ADHandler.actioncoreDescription[actioncore]['action_roles']
    
    @staticmethod
    def get_required_roles_based_on_actioncore(actioncore):
        ADHandler = ActioncoreDescriptionHandler
        if ADHandler.actioncoreDescription == {}:
            ADHandler.loadActioncoreDescription()
        return ADHandler.actioncoreDescription[actioncore]['required_action_roles']
    
    @staticmethod
    def roles():
        ActioncoreDescriptionHandler.loadActioncoreDescription()
        return set(reduce(list.__add__, [ActioncoreDescriptionHandler.actioncoreDescription[ac]['action_roles'] for ac in ActioncoreDescriptionHandler.actioncoreDescription]))







