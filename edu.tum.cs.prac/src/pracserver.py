#!/usr/bin/env python

from network import JSONRPCService, jsonremote
import web
import os
import java
from actioncore import PRAC
from actioncore.inference import *
import jpype
import json
from nltk.corpus import wordnet as wn

urls = (
    '/prac/', 'pracserver',
    '/prac',  'pracserver',
    '/(.*)', 'static'
    )

class pracserver:
    def POST(self):
        reply = prac_server(web.webapi.data())
        return reply

static_location = os.path.join('.', 'pracviz', 'src', 'output')

class static:
    def GET(self, name):
    
        if name == '':
            name = 'pracviz.html'
    
        ext = name.split(".")[-1]
    
        cType = {
            "js":"application/x-javascript",
            "txt":"text/plain",
            "css":"text/css",
            "html":"text/html",
            "png":"images/png",
            "jpg":"image/jpeg",
            "gif":"image/gif",
            "ico":"image/x-icon",
            "svg": "image/svg+xml"}
    
#         static_location = os.path.join('.', 'pracviz', 'src', 'output')
        if name in os.listdir(static_location):
            web.header("Content-Type", cType[ext])
            return open('%s/%s' % (static_location, name), "rb").read()
        else:
            web.notfound()


class PracRpcService(JSONRPCService):
    
    def __init__(self):
        JSONRPCService.__init__(self)
        
prac_server = PracRpcService()
prac_server.prac = PRAC()


@jsonremote(prac_server)
def action_cores(request, msg):
    print prac_server.prac.action_cores
    return prac_server.prac.action_cores.keys()

@jsonremote(prac_server)
def get_syntactic_atoms(request, msg):
    if not jpype.isThreadAttachedToJVM():
        jpype.attachThreadToJVM()
    print msg
    prac_server.pracinit = PRACInit('Flipping')
    prac_server.result = PRACResult()
    prac_server.pracinit(msg) >> prac_server.result
#     db = pracinit.pracinference.databases['core']    
    syntactic_feat_graph = prac_server.result.pracinference.to_syntactic_graph()
    print json.dumps(syntactic_feat_graph)
    return syntactic_feat_graph

@jsonremote(prac_server)
def get_wordsenses(request, msg):
    senses_graph = prac_server.result.pracinference.to_word_senses()
    print senses_graph
    return senses_graph

@jsonremote(prac_server)
def get_possible_roles(request, msg):
    poss_roles = prac_server.result.pracinference.to_possible_roles()
    print poss_roles
    return poss_roles

@jsonremote(prac_server)
def get_senses_and_roles(request, msg):
    sensesAndRoles = prac_server.result.pracinference.to_senses_and_roles()
    print sensesAndRoles
    return sensesAndRoles

@jsonremote(prac_server)
def get_removed_nodes(request, msg):
    removeNodes = prac_server.result.pracinference.to_inapplicable_nodes()
    print removeNodes
    return removeNodes

@jsonremote(prac_server)
def get_missing_roles(request, msg):
    prac_server.actionroles = actionroles
    prac_server.result >> actionroles >> prac_server.result
    missingRoles = prac_server.result.pracinference.to_missing_roles()
    print missingRoles
    return missingRoles

@jsonremote(prac_server)
def get_possible_missing_roles(request, msg):
    possibleMissingRoles = prac_server.result.pracinference.to_possible_missing_roles()
    print possibleMissingRoles
    return possibleMissingRoles

@jsonremote(prac_server)
def get_missing_role_senses(request, msg):
    possibleMissingRoles = prac_server.result.pracinference.to_missing_role_senses()
    print possibleMissingRoles
    return possibleMissingRoles

@jsonremote(prac_server)
def get_no_variables(request, msg):
    count = len(prac_server.result.pracinference.mlns[msg].mrf.gndAtoms)
    return {'no_variables': count}

@jsonremote(prac_server)
def get_no_gnd_formulas(request, msg):
    mln = prac_server.result.pracinference.mlns[msg]
    count = 0
    for f in mln.formulas:
        vars = f.getVariables(mln.mrf)
        if len(vars) == 0: continue
        gndFormulaCount = reduce(lambda x, y: x*y, [len(mln.mrf.domains[f.getVarDomain(v, mln.mrf)]) for v in vars])
        count += gndFormulaCount
    return {'no_gnd_formulas': count}

# start me!
if __name__ == "__main__":
    java.startJvm()
    print 'running in ', static_location
    web.application(urls, globals()).run()
    java.shutdownJvm()