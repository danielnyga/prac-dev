from pracWEB.pracinit import pracApp
from prac.inference import PRACInference, PRACInferenceStep
from mln import readMLNFromFile, readDBFromFile
from mln.database import readDBFromString
from mln.methods import InferenceMethods
from mln.mln import readMLNFromString
from flask import render_template, redirect, request, jsonify, url_for
from pracWEB.pages.fileupload import upload
from pracWEB.pages.utils import updateKBList, updateMLNList, updateEvidenceList, LOGICS, FILEDIRS, getFileContent, save_kb, add_wn_similarities
import os, sys
import pickle
import StringIO
import logging
from flask.globals import session
import json
from pracWEB.pages.views import ensure_prac_session
from pracutils.RolequeryHandler import RolequeryHandler
from prac.core import PRAC
from prac.wordnet import WordNet

INFMETHODS = [(InferenceMethods.byName(method),method) for method in InferenceMethods.name2value]


@pracApp.app.route('/_pracinfer_step', methods=['POST', 'GET'])
def _pracinfer_step():
    log = logging.getLogger(__name__)
    print pracApp.app.session_store
    pracsession = ensure_prac_session(session)
    prac = pracsession.prac

    if request.method == 'POST':
        data = json.loads(request.get_data())
        pracsession.count = 1
        log.info('starting new PRAC inference on "%s"' % data['sentence'])
#         prac = PRAC()
#         prac.wordnet = WordNet(concepts=None)
        pracsession.prac = prac
        infer = PRACInference(prac, [data['sentence']])
        parser = prac.getModuleByName('nl_parsing')
        prac.run(infer, parser)
        pracsession.infer = infer
        pracsession.synPreds = parser.mln.predicates
        pracsession.leaveSynPreds = True
    else:
        if pracsession.infer.next_module() is not None :
            module = prac.getModuleByName(pracsession.infer.next_module())
            pracsession.lastModule = module
            print 'running', module.name
            prac.run(pracsession.infer,module)
        else:
            result = []
            for db in pracsession.infer.inference_steps[-1].output_dbs:
                db.write(sys.stdout, color=True)
#                 print 'rolequerydingen'
                RolequeryHandler.queryRolesBasedOnAchievedBy(db).printEvidence()
#                 print '/rolequerydingen'
            #     db.write(sys.stdout, color=True)
            #     evPreds = list(set([x.split('(')[0] for x in db.evidence.keys()]))
            #     for q in db.query("action_core(?w, ?ac)"):
            #         ac = q['?ac']
            #         kb = pracsession.lastModule.load_pracmt(ac+"Transformation")
            #         queryPredicates = kb.query_params['queries'].split(",")
            #         print queryPredicates

            #         # add querypredicates
            #         for role in queryPredicates:
            #             print 'querying {}(?w, {}) ^ has_sense(?w, ?s)'.format(role, ac)
            #             for q2 in db.query("{}(?w, {}) ^ has_sense(?w, ?s)".format(role, ac)):
            #                 result.append({'source': ac, 'target': q2['?s'] , 'value': role , 'arcStyle': 'strokegreen'})

            #         # add other predicates
            #         for s in evPreds:
            #             if s in pracsession.synPreds.keys() + queryPredicates + ['has_sense']: 
            #                 evPreds.remove(s)
            #             else:
            #                 for q3 in db.query("{}(?w, ?w2) ^ has_sense(?w, ?s)".format(s)):
            #                     result.append({'source': q3['?w'], 'target': q3['?w2'] , 'value': s , 'arcStyle': 'strokegreen'})
            print 'result: ', result
            return jsonify( {'result': result, 'finish': True} )

    result = []
    for db in pracsession.infer.inference_steps[-1].output_dbs:
        db.write(sys.stdout, color=True)
        _grammar = db.mln.logic.grammar
        for atom in db.evidence:
            a_tuple = _grammar.parseLiteral(atom)
            if not db.evidence[atom] == 1: continue
            if a_tuple[1] in pracsession.synPreds.keys() and not pracsession.leaveSynPreds: continue
            print 'rolequerydingen'
            # RolequeryHandler.queryRolesBasedOnAchievedBy(db).printEvidence()
            print '/rolequerydingen'        
            if 'null' in a_tuple[2] or a_tuple[1] == 'is_a': continue
            if len(a_tuple[2]) == 2:
                result.append({'source': a_tuple[2][0], 'target': a_tuple[2][1] , 'value': a_tuple[1] , 'arcStyle': 'strokegreen'})
            else:
                result.append({'source': a_tuple[2][2], 'target': a_tuple[2][0] , 'value': a_tuple[1] , 'arcStyle': 'strokegreen'})
                result.append({'source': a_tuple[2][2], 'target': a_tuple[2][1] , 'value': a_tuple[1] , 'arcStyle': 'strokegreen'})
    pracsession.leaveSynPreds = False
    return jsonify( {'result': result, 'finish': False} )

 
# def infer(data, files):
#     if data['module'] in pracApp.prac.moduleManifestByName: # call module's inference method
#         print 'Running Inference for module ', data['module']
#         infer = PRACInference(pracApp.prac, [])
#         module = pracApp.prac.getModuleByName(str(data['module']))
#         inferenceStep = PRACInferenceStep(infer, module)
# 
#         mln = readMLNFromString(str(data['mln']),str( data['logic']))
#         
#         trainingDBs = readDBFromString(mln, str(data['evidence']), ignoreUnknownPredicates=True)
#         inferenceStep.output_dbs = trainingDBs
#         infer.inference_steps.append(inferenceStep)
# 
#         if 'kb' in data:
#             kb = module.load_pracmt(str(data['kb']))
#         else:
#             kb = module.load_pracmt('default')
# 
#         params = {}
#         params['queries'] = str(data['queries'])
#         params['method'] = str(data['method'])
#         params['cwPreds'] = str(data['cwPreds'])
#         params['closedWorld'] = (1 if 'closedWorld' in data else 0)
#         params['useMultiCPU'] = (1 if 'useMultiCPU' in data else 0)
#         params['logic'] = str(data['logic'])
#         params.update(eval("dict({})".format(str(data['parameters']))))
# 
#         kb.query_params = params
#         kb.set_querymln(str(data['mln']), path=os.path.join(module.module_path, 'mln'))
#         kb.dbs = list(readDBFromString(kb.query_mln, str(data['evidence'])))
# 
#         pracApp.prac.run(infer, module, mln=mln, kb=kb)
#         step = infer.inference_steps[-1]
# 
#         if 'saveKB' in data:
#             if 'kbName' in data:
#                 module.save_pracmt(kb, str(data['kbName']))
#             else:
#                 module.save_pracmt(kb, str(data['kb']))
# 
#     else: # inference without module (no WN)
#         print 'Running Inference w/o module'
