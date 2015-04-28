from pracWEB.pracinit import pracApp
from prac.inference import PRACInference, PRACInferenceStep
from mln import readMLNFromFile, readDBFromFile
from mln.database import readDBFromString
from mln.methods import InferenceMethods
from mln.mln import readMLNFromString
from flask import render_template, redirect, request, jsonify, url_for
from wtforms import BooleanField, TextField, TextAreaField, validators, SelectField, FileField, SubmitField
from flask_wtf import Form
from pracWEB.pages.fileupload import upload
from pracWEB.pages.utils import updateKBList, updateMLNList, updateEvidenceList, LOGICS, FILEDIRS, getFileContent, save_kb, add_wn_similarities
import os, sys
import pickle
import StringIO
import logging
from flask.globals import session
import json

INFMETHODS = [(InferenceMethods.byName(method),method) for method in InferenceMethods.name2value]


@pracApp.app.route('/_pracinfer_step', methods=['POST', 'GET'])
def _pracinfer_step():
    log = logging.getLogger(__name__)
    print pracApp.app.session_store
    pracsession = pracApp.app.session_store[session]
    prac = pracsession.prac
    if request.method == 'POST':
        data = json.loads(request.get_data())
        pracsession.count = 1
        log.info('starting new PRAC inference on "%s"' % data['sentence'])
        infer = PRACInference(prac, [data['sentence']])
        parser = prac.getModuleByName('nl_parsing')
        prac.run(infer, parser)
        return 'the new graph'
    else:
        if pracsession.count < 5:
            pracsession.count += 1
            return 'the very new graph'
        else:
            return 'finish'
        
    
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
# 
#     result = {}
#     stpno = 0
#     for stp in infer.inference_steps:
#         stepx = []
#         for db in stp.output_dbs:
#             for ek in db.evidence:
#                 e = db.evidence[ek]
#                 src = ek.split('(')[1].split(',')[0]
#                 tar = ek.split('(')[1].split(',')[1].split(')')[0]
#                 val = ek.split('(')[0] # db.evidence[ek]?\
#                 arcStyle = 'default'
#                 stepx.append({'source': src, 'target': tar , 'value': val , 'arcStyle': arcStyle})
#         result[stpno] = stepx
#         stpno += 1
#     return result
