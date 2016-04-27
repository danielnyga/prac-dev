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

import os
from prac.pracutils.ActioncoreDescriptionHandler import \
    ActioncoreDescriptionHandler
from pracmln import Database
from pracmln.praclog import logger


class RolequeryHandler(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    actioncoreDescriptionFilePath = os.path.join(PRAC_HOME, 'models', 'actioncores.yaml')
    actioncoreDescription = {}
    
    @staticmethod
    def queryRolesAndSensesBasedOnActioncore(db):
        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database 
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
        
        roles_db = RolequeryHandler.queryRoles(actioncore,db)
        
        inferred_roles_set = set()
        roles_dict = {}
        
        #Get inferred roles and the corresponding senses
        for atom, truth in sorted(roles_db.evidence.iteritems()):
            _ , predname, args = roles_db.mln.logic.parse_literal(atom)
            if truth == 1.0:
                inferred_roles_set.add(predname)
                for sense_query in db.query('has_sense({},?s)'.format(args[0])):
                    roles_dict[predname] = sense_query['?s']
        
        return roles_dict
    
    @staticmethod
    def queryRolesBasedOnAchievedBy(db):
        actioncore = ""
        #It will be assumed that there is only one true achieved_by predicate per database 
        for q in db.query("achieved_by(?w,?ac)"):
            actioncore = q["?ac"]
        
        return RolequeryHandler.queryRoles(actioncore,db)
    
    @staticmethod
    def queryRolesBasedOnActioncore(db):
        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database 
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]

        return RolequeryHandler.queryRoles(actioncore,db)
    
    @staticmethod
    def queryRoles(actioncore,db):
        db_ = Database(db.mln)
        rolePredicates = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
        for p in rolePredicates:
            query = RolequeryHandler.roleQueryBuilder(actioncore, p, db.mln.predicate(p).argdoms)
            for q in db.query(query, thr=1):
                for var, val in q.iteritems():
                    q_ = query.replace(var,val)
                    db_ << q_
        return db_

    @staticmethod
    def roleQueryBuilder(actioncore,predicate, domainList):
        # assuming that the role predicates are always of the form
        # predname(?x, actioncore)
        return '{}(?{},{})'.format(predicate,domainList[0],actioncore)
    
    '''
    Returns a dict with the roles as key and senses as values
    '''
    @staticmethod
    def query_roles_and_senses_based_on_action_core(db):
        roles_dict = {}
        #It will be assumed that there is only one true action_core predicate per database 
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
            roles_db = RolequeryHandler.queryRoles(actioncore,db)
            
            for atom, truth in sorted(roles_db.evidence.iteritems()):
                _ , predname, args = roles_db.mln.logic.parse_literal(atom)
                if truth == 1.0:
                    for sense_query in db.query('has_sense({},?s)'.format(args[0])):
                        roles_dict[predname] =  sense_query['?s']
        return roles_dict