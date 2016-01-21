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
from pracutils.ActioncoreDescriptionHandler import ActioncoreDescriptionHandler
from mln.database import Database

class RolequeryHandler(object):
    PRAC_HOME = os.environ['PRAC_HOME']
    actioncoreDescriptionFilePath = os.path.join(PRAC_HOME, 'models', 'actioncores.yaml')
    actioncoreDescription = {}
    
    @staticmethod
    def queryRolesBasedOnAchievedBy(db,truth=1):
        actioncore = ""
        #It will be assumed that there is only one true achieved_by predicate per database 
        for q in db.query("achieved_by(?w,?ac)"):
            actioncore = q["?ac"]
        
        return RolequeryHandler.queryRoles(actioncore,db,truth)
    
    @staticmethod
    def queryRolesBasedOnActioncore(db,truth=1):
        actioncore = ""
        #It will be assumed that there is only one true action_core predicate per database 
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
        
        return RolequeryHandler.queryRoles(actioncore,db,truth)
    
    @staticmethod
    def queryRoles(actioncore,db,truth=1):
        db_ = Database(db.mln)
        rolePredicates = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
        
        for p in rolePredicates:
            query = RolequeryHandler.roleQueryBuilder(actioncore,p, db.mln.predicates[p])
            for q in db.query(query, truthThreshold=truth):
                for var, val in q.iteritems():
                    q_ = query.replace(var,val)
                    db_.addGroundAtom(q_,db.evidence[q_])
        return db_
    
    @staticmethod
    def roleQueryBuilder(actioncore,predicate, domainList):
        query = predicate+'('
        
        if domainList[0].lower() == 'actioncore':
            query += actioncore
        else:
            query += "?"+domainList[0]
            
        if len(domainList) == 1:
            query += ")"
        else:
            i = 1
            for d in domainList[1:]:
                query += ","
                if d.lower() == 'actioncore':
                    query += actioncore
                else:
                    query += "?"+str(i)+d
            query += ")"
        return query
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
                _ , predname, args = roles_db.mln.logic.parseLiteral(atom)
                if truth == 1.0:
                    for sense_query in db.query('has_sense({},?s)'.format(args[0])):
                        roles_dict[predname] =  sense_query['?s']
        return roles_dict