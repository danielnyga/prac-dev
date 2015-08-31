from flask import request, jsonify
import sys
import StringIO
import logging
from flask.globals import session
import json
from prac.core.inference import PRACInference
from prac.pracutils import ActioncoreDescriptionHandler
from pracweb.gui.pages.utils import ensure_prac_session
from pracmln import Database
from pracweb.gui.app import pracApp


@pracApp.app.route('/prac/_pracinfer_step', methods=['POST', 'GET'])
def _pracinfer_step():
    log = logging.getLogger(__name__)
    pracsession = ensure_prac_session(session)
    prac = pracsession.prac

    # first inference step (nl parsing)
    if request.method == 'POST':
        data = json.loads(request.get_data())
        pracsession.count = 1
        log.info('starting new PRAC inference on "%s"' % data['sentence'])
        pracsession.prac = prac
        infer = PRACInference(prac, [data['sentence']])
        parser = prac.getModuleByName('nl_parsing')
        prac.run(infer, parser)
        evidence = infer.inference_steps[-1].output_dbs
        pracsession.infer = infer
        pracsession.synPreds = parser.mln.predicates
        pracsession.leaveSynPreds = True
    else:
        step = pracsession.infer.inference_steps[-1]
        evidence = step.output_dbs

        if pracsession.infer.next_module() is not None :
            module = prac.getModuleByName(pracsession.infer.next_module())
            prac.run(pracsession.infer,module)
            if module.name == 'senses_and_roles':
                pracsession.sar_step = pracsession.infer.inference_steps[-1]
        # final step used to generate final graph structure
        else:
            result = []
            finaldb = Database(pracsession.prac.mln)
            for step in pracsession.infer.inference_steps:
                for db in step.output_dbs:
                    for atom, truth in db.evidence.iteritems():
                        if truth == 0: continue
                        _, predname, args = pracsession.prac.mln.logic.parse_literal(atom)
                        if predname in ActioncoreDescriptionHandler.roles().union(['has_sense', 'action_core', 'achieved_by']):
                            finaldb.addGroundAtom(atom)
            # add all roles and word sense links
            for res in finaldb.query('action_core(?w, ?a) ^ has_sense(?w, ?s)'):
                actioncore = res['?a']
                sense = res['?s']
                result.append({'source': actioncore, 'target': sense , 'value': 'is_a' , 'arcStyle': 'strokegreen'})
                
                roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
                for role in roles:
                    for res in db.query('%s(?w, %s) ^ has_sense(?w, ?s)' % (role, actioncore)):
                        sense = res['?s']
                        result.append({'source': actioncore, 'target': sense , 'value': role , 'arcStyle': 'strokegreen'})
            for res in finaldb.query('achieved_by(?a1, ?a2)'):
                a1 = res['?a1']
                actioncore = res['?a2']
                result.append({'source': a1, 'target': actioncore , 'value': 'achieved_by' , 'arcStyle': 'strokegreen'})
                roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(actioncore)
                for role in roles:
                    for res in db.query('%s(?w, %s) ^ has_sense(?w, ?s)' % (role, actioncore)):
                        sense = res['?s']
                        result.append({'source': actioncore, 'target': sense , 'value': role , 'arcStyle': 'strokegreen'})

            # store current inference step if an executable plan exists
            if hasattr(pracsession.infer.inference_steps[-1], 'executable_plans'):
                pracsession.old_infer = pracsession.infer
            else:
                pracsession.old_infer = []

            # retrieve settings for executed pracmodule
            step = pracsession.infer.inference_steps[-1]
            if hasattr(step, 'applied_kb'):
                settings = _get_settings(step.module, step.applied_kb, evidence)
            else:
                settings = None

            # finally delete inference object
            delattr(pracsession, 'infer')

            return jsonify( {'result': result, 'finish': True, 'settings': settings } )

    # generate graph links
    result = []
    for db in pracsession.infer.inference_steps[-1].output_dbs:
        db.write(sys.stdout, color=True)
        _grammar = db.mln.logic.grammar
        for atom in db.evidence:
            a_tuple = _grammar.parse_literal(atom)
            if not db.evidence[atom] == 1: continue
            if a_tuple[1] in pracsession.synPreds and not pracsession.leaveSynPreds: continue
            if 'null' in a_tuple[2] or a_tuple[1] == 'is_a': continue
            if len(a_tuple[2]) == 2:
                result.append({'source': a_tuple[2][0], 'target': a_tuple[2][1] , 'value': a_tuple[1] , 'arcStyle': 'strokegreen'})
            else:
                result.append({'source': a_tuple[2][2], 'target': a_tuple[2][0] , 'value': a_tuple[1] , 'arcStyle': 'strokegreen'})
                result.append({'source': a_tuple[2][2], 'target': a_tuple[2][1] , 'value': a_tuple[1] , 'arcStyle': 'strokegreen'})

    pracsession.leaveSynPreds = False

    # retrieve settings for executed pracmodule
    step = pracsession.infer.inference_steps[-1]
    if hasattr(step, 'applied_kb'):
        settings = _get_settings(step.module, step.applied_kb, evidence)
    else:
        settings = _get_settings(step.module, None, evidence)

    return jsonify( { 'result': result, 'finish': False, 'settings': settings } )


@pracApp.app.route('/prac/_pracinfer_get_next_module', methods=['GET'])
def _pracinfer_get_next_module():
    pracsession = ensure_prac_session(session)
    if hasattr(pracsession, 'infer'):
        modulename = pracsession.infer.next_module()
        return 'None' if modulename is None else modulename
    else:
        return 'nl_parsing'


def _get_settings(module, kbname, evidence):
    settings = {}
    settings['module'] = module.name
    settings['mln'] = ''

    # evidence is either text or list of dbs
    if type(evidence) is unicode:
        settings['evidence'] = evidence
    else:
        dbStr = StringIO.StringIO()
        for db in evidence:
            db.write(dbStr, color=False, bars=False)
        settings['evidence'] = dbStr.getvalue()
    
    # if knowledge base exists, read settings
    if kbname is not None:
        kb = module.load_prac_kb(kbname)
        mlnstr = StringIO.StringIO()
        kb.query_mln.write(mlnstr, color=False)
        settings.update(kb.config.config)
        settings['mln'] = mlnstr.getvalue()

    return settings

def format_cram_string(cramString):
    l = {}
    newstring = ''
    lines = cramString.split('\n')

    for line in lines:
        ts = line.split('\t!')
        nline = ''.join(ts)
        tabcnt = nline.count('\t')

        if len(ts) > 1:
            for t, tn in enumerate(ts):
                l[t+tabcnt] = len(''.join([x.strip('\t') for x in ts[:t]])) + l.get(tabcnt, 0)
        newstring += ' '*l.get(tabcnt, 0) + nline.strip() + '\n'
    return newstring

@pracApp.app.route('/prac/_get_cram_plan', methods=['GET'])
def _get_cram_plan():
    pracsession = ensure_prac_session(session)
    cramPlans = pracsession.old_infer.inference_steps[-1].executable_plans

    return jsonify( {'plans': [format_cram_string(cp) for cp in cramPlans] })


@pracApp.app.route('/prac/_pracinfer_get_cond_prob', methods=['GET'])
def _get_cond_prob():
    pracsession = ensure_prac_session(session)
    ratio = 1
    png = ''
    if hasattr(pracsession, 'infer'):
        inf_step = pracsession.infer.inference_steps[-1]
        if hasattr(inf_step, 'png'):
            png, ratio = inf_step.png
    return jsonify( {'ratio': ratio, 'img': png })


@pracApp.app.route('/prac/_get_role_distributions', methods=['GET'])
def _get_role_distributions():
    pracsession = ensure_prac_session(session)
    module = pracsession.prac.getModuleByName('senses_and_roles')
    role_dists = module.role_distributions(pracsession.sar_step)
    return jsonify( {'distributions': role_dists })

