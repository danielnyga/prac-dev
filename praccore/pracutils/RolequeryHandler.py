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
    def queryRoles(actioncore,db):
        db_ = Database(db.mln)
        rolePredicates = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
        
        for p in rolePredicates:
            query = RolequeryHandler.roleQueryBuilder(actioncore,p, db.mln.predicates[p])
            for q in db.query(query, truthThreshold=1):
                for var, val in q.iteritems():
                    query = query.replace(var,val)
                db_.addGroundAtom(query)
                
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