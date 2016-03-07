import traceback
import ctypes
import multiprocessing as mp
from flask import request, jsonify
from StringIO import StringIO
from flask.globals import session
from threading import Thread
import json
import logging
from prac.core.inference import PRACInference
from prac.pracutils.ActioncoreDescriptionHandler import \
    ActioncoreDescriptionHandler
from pracmln.mln.util import out
from pracmln.praclog import logger
from pracweb.gui.pages.buffer import RequestBuffer
from pracweb.gui.pages.utils import ensure_prac_session
from pracmln import Database
from pracweb.gui.app import pracApp
from prac.core.wordnet import WordNet
from prac.core.wordnet_online import WordNet as AcatWordnet


log = logger(__name__)
wn = WordNet(concepts=None)
awn = AcatWordnet(concepts=None)


@pracApp.app.route('/prac/_start_inference', methods=['POST', 'GET'])
def start_inference():
    pracsession = ensure_prac_session(session)
    method = request.method
    if method == 'POST':
        data = json.loads(request.get_data())
    else:
        data = None
    pracsession.infbuffer = RequestBuffer()
    t = Thread(target=_pracinfer, args=(pracsession, 180, method, data))
    t.start()
    pracsession.infbuffer.waitformsg()
    return jsonify(pracsession.infbuffer.content)


@pracApp.app.route('/prac/_get_status', methods=['POST'])
def getinfstatus():
    pracsession = ensure_prac_session(session)
    pracsession.infbuffer.waitformsg()
    return jsonify(pracsession.infbuffer.content)


def _pracinfer(pracsession, timeout, method, data):
    prac = pracsession.prac
    prac.log.setLevel(logging.ERROR)
    #prac.mln.verbose = False # does nothing
    logmsg = ''
    result = []
    settings = {}
    finish = False
    msg = ''
    pracsession.log.info('STARTING INFERENCE STEP')
    pracsession.infbuffer.setmsg({'message': '', 'status': False})
    try:
        if method == 'POST' or (hasattr(pracsession, 'infer') and pracsession.infer.next_module() is not None):
            # any step except first and last
            if hasattr(pracsession, 'infer') \
                    and pracsession.infer.next_module() is not None:
                # no executable plan has been found so far
                if pracsession.infer.next_module() is not None:
                    module = prac.getModuleByName(
                        pracsession.infer.next_module())
                    pracsession.log.info('Running Module {}'.format(module.name))

                    t = Thread(target=prac.run, args=(pracsession.infer, module))
                    t.start()
                    threadid = t.ident
                    t.join(timeout)  # wait until either thread is done or time is up

                    if t.isAlive():
                        # stop inference and raise TimeoutError locally
                        ctypes.pythonapi.PyThreadState_SetAsyncExc(
                            ctypes.c_long(threadid),
                            ctypes.py_object(SystemExit))
                        raise mp.TimeoutError

                    if module.name == 'senses_and_roles':
                        pracsession.sar_step = \
                            pracsession.infer.inference_steps[-1]

            # first inference step (nl parsing)
            else:
                if hasattr(pracsession, 'infer'):
                    delattr(pracsession, 'infer')

                if data['acatontology']:
                    prac.wordnet = awn
                else:
                    prac.wordnet = wn

                pracsession.log.info('starting new PRAC inference on "{}"'.format(
                    data['sentence']))
                pracsession.prac = prac
                infer = PRACInference(prac, [data['sentence']])
                parser = pracsession.parser

                t = Thread(target=prac.run, args=(infer, parser))
                t.start()
                threadid = t.ident
                # wait until either thread is done or time is up
                t.join(timeout)

                if t.isAlive():
                    # stop inference and raise TimeoutError locally
                    ctypes.pythonapi.PyThreadState_SetAsyncExc(
                        ctypes.c_long(threadid),
                        ctypes.py_object(SystemExit))
                    raise mp.TimeoutError

                pracsession.infer = infer
                pracsession.synPreds = parser.mln.predicates
                pracsession.leaveSynPreds = True

            step = pracsession.infer.inference_steps[-1]
            evidence = step.output_dbs
            # generate graph links
            result = generate_graph_links(pracsession, evidence)
            pracsession.leaveSynPreds = False
        # final step used to generate final graph structure
        else:
            pracsession.log.info('Finalizing result...')
            step = pracsession.infer.inference_steps[-1]
            evidence = step.output_dbs
            finish = True

            result = generate_final_links(pracsession)

            # store current inference step if an executable plan exists
            if hasattr(step, 'executable_plans'):
                pracsession.old_infer = pracsession.infer

            # finally delete inference object
            delattr(pracsession, 'infer')

        # retrieve settings for executed pracmodule
        if hasattr(step, 'applied_settings'):
            settings = _get_settings(step.module,
                                     step.applied_settings,
                                     evidence)
        else:
            settings = _get_settings(step.module, None, evidence)
    except SystemExit:
        pracsession.log.error('Cancelled...')
        msg = 'Cancelled!\nCheck log for more information.'
    except mp.TimeoutError:
        pracsession.log.error('Timeouterror! '
                        'Inference took more than {} seconds. '
                        'Increase the timeout and try again.'.format(timeout))
        msg = 'Timeout!'
    except:
        traceback.print_exc()
        traceback.print_exc(file=pracsession.stream)
        msg = 'Failed!\nCheck log for more information.'
    finally:
        pracsession.loghandler.flush()
        pracsession.log.info('\n')
        value = pracsession.stream.getvalue()
        if finish:
            pracsession.stream.truncate(0)
            msg = 'Success!'
        try:
            logmsg += u'\n%s' % unicode(value, 'utf-8')
        except UnicodeError:
            traceback.print_exc()
            logmsg += u'\n%s' % repr(value)

        pracsession.infbuffer.setmsg({'result': result,
                                      'finish': finish,
                                      'status': True,
                                      'settings': settings,
                                      'log': logmsg,
                                      'message': msg})


@pracApp.app.route('/prac/_pracinfer_get_next_module', methods=['GET'])
def _pracinfer_get_next_module():
    pracsession = ensure_prac_session(session)
    if hasattr(pracsession, 'infer'):
        modulename = pracsession.infer.next_module()
        return 'None' if modulename is None else modulename
    else:
        return 'nl_parsing'


@pracApp.app.route('/prac/_get_cram_plan', methods=['GET'])
def _get_cram_plan():
    pracsession = ensure_prac_session(session)
    if pracsession.old_infer is not None:
        cramplans = pracsession.old_infer.inference_steps[-1].executable_plans
    else:
        cramplans = []

    return jsonify({'plans': [format_cram_string(cp) for cp in cramplans]})


@pracApp.app.route('/prac/_pracinfer_get_cond_prob', methods=['GET'])
def _get_cond_prob():
    pracsession = ensure_prac_session(session)
    ratio = 1
    png = ''
    if hasattr(pracsession, 'infer'):
        inf_step = pracsession.infer.inference_steps[-1]
        if hasattr(inf_step, 'png'):
            png, ratio = inf_step.png
    return jsonify({'ratio': ratio, 'img': png})


@pracApp.app.route('/prac/_get_role_distributions', methods=['GET'])
def _get_role_distributions():
    pracsession = ensure_prac_session(session)
    prac = pracsession.prac
    prac.wordnet = WordNet()
    module = prac.getModuleByName('senses_and_roles')
    role_dists = module.role_distributions(pracsession.sar_step)
    return jsonify({'distributions': role_dists})


def _get_settings(module, appliedsettings, evidence):
    settings = {'module': module.name, 'mln': ''}

    # if settings base exists, read settings
    if appliedsettings is not None:
        mlnstr = StringIO()
        appliedsettings.get('mln').write(mlnstr, color=False)
        settings.update(appliedsettings)
        del settings['db']
        settings['mln'] = mlnstr.getvalue()

    # evidence is either text or list of dbs
    if type(evidence) is unicode:
        settings['evidence'] = evidence
    else:
        dbstr = StringIO()
        for db in evidence:
            db.write(dbstr, color=False, bars=False)
        settings['evidence'] = dbstr.getvalue()

    return settings


def format_cram_string(cramstring):
    l = {}
    newstring = ''
    lines = cramstring.split('\n')

    for line in lines:
        ts = line.split('\t!')
        nline = ''.join(ts)
        tabcnt = nline.count('\t')

        if len(ts) > 1:
            for t, tn in enumerate(ts):
                l[t + tabcnt] = len(
                    ''.join([x.strip('\t') for x in ts[:t]])) + l.get(tabcnt,
                                                                      0)
        newstring += ' ' * l.get(tabcnt, 0) + nline.strip() + '\n'
    return newstring


def generate_graph_links(pracsession, evidence):
    result = []
    for db in evidence:
        db.write(pracsession.stream, color=False, bars=False)
        _grammar = db.mln.logic.grammar
        for atom in db.evidence:
            a_tuple = _grammar.parse_literal(atom)
            if not db.evidence[atom] == 1:
                continue
            if a_tuple[1] in pracsession.synPreds \
                    and not pracsession.leaveSynPreds:
                continue
            if 'null' in a_tuple[2] or a_tuple[1] == 'is_a':
                continue

            doms = pracsession.prac.mln.predicate(a_tuple[1]).argdoms
            if len(a_tuple[2]) == 2:
                if doms[0] in ['concept', 'sense']:
                    stext = wn.synset(a_tuple[2][0]).definition
                else:
                    stext = ''
                if doms[1] in ['concept', 'sense']:
                    ttext = wn.synset(a_tuple[2][1]).definition
                else:
                    ttext = ''
                result.append(
                    {'source': {'name': a_tuple[2][0], 'text': stext},
                     'target': {'name': a_tuple[2][1], 'text': ttext},
                     'value': a_tuple[1], 'arcStyle': 'strokegreen'})
            else:
                if doms[0] in ['concept', 'sense']:
                    stext = wn.synset(a_tuple[2][0]).definition
                else:
                    stext = ''
                if doms[1] in ['concept', 'sense']:
                    ttext = wn.synset(a_tuple[2][1]).definition
                else:
                    ttext = ''
                result.append(
                    {'source': {'name': a_tuple[2][2], 'text': stext},
                     'target': {'name': a_tuple[2][0], 'text': ttext},
                     'value': a_tuple[1], 'arcStyle': 'strokegreen'})
                if doms[2] in ['concept', 'sense']:
                    ttext = wn.synset(a_tuple[2][1]).definition
                else:
                    ttext = ''
                result.append(
                    {'source': {'name': a_tuple[2][2], 'text': stext},
                     'target': {'name': a_tuple[2][1], 'text': ttext},
                     'value': a_tuple[1], 'arcStyle': 'strokegreen'})
    return result

# side effect: fill in actioncores list to pass to CRAM
def generate_final_links(pracsession):
    result = []

    finaldb = Database(pracsession.prac.mln)
    for step in pracsession.infer.inference_steps:
        for db in step.output_dbs:
            for atom, truth in db.evidence.iteritems():
                if truth == 0:
                    continue
                _, predname, _ = pracsession.prac.mln.logic.parse_literal(
                    atom)
                if predname in ActioncoreDescriptionHandler.roles().union(
                        ['has_sense', 'action_core', 'achieved_by']):
                    finaldb << (atom, truth)

    # want something nice like this in action_cores:
    # action_cores:
    #   - action_core_name: 'Starting'
    #     action_roles:
    #       - role_name: obj_to_be_started
    #         role_value: centrifuge.n.01
    #   - action_core_name: 'TurningOnElectricalDevice'
    #     action_roles:
    #       - role_name: device
    #         role_value: centrifuge.n.01
    actioncores = []
    # add all roles and word sense (is_a) links
    for res in finaldb.query(
            'action_core(?w, ?a) ^ has_sense(?w, ?s)'):
        actioncore = res['?a']
        # wordnet sense
        sense = res['?s']
        result.append({'source': {'name': actioncore, 'text': ''},
                       'target': {'name': sense,
                                  'text': wn.synset(sense).definition},
                       'value': 'is_a', 'arcStyle': 'strokegreen'})

        actioncore_dict = {}
        actioncore_dict['action_core_name'] = actioncore
        actioncore_dict['action_roles'] = []

        roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(
            actioncore)
        for role in roles:
            # we want to know whats missing, too
            sense = None
            for dbres in finaldb.query('{}(?w, {}) ^ has_sense(?w, ?s)'.format(
                    role, actioncore)):
                sense = dbres['?s']
                result.append(
                    {'source': {'name': actioncore, 'text': ''},
                     'target': {'name': sense,
                                'text': wn.synset(sense).definition},
                     'value': role,
                     'arcStyle': 'strokegreen'})

            actioncore_dict['action_roles'].append({'role_name': role, 'role_value': sense})

        actioncores.append(actioncore_dict)



    for finaldbres in finaldb.query('achieved_by(?a1, ?a2)'):
        a1 = finaldbres['?a1']
        actioncore = finaldbres['?a2']
        result.append({'source': {'name': a1, 'text': ''},
                       'target': {'name': actioncore, 'text': ''},
                       'value': 'achieved_by',
                       'arcStyle': 'strokegreen'})

        actioncore_dict = {}
        actioncore_dict['action_core_name'] = actioncore
        actioncore_dict['action_roles'] = []

        roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(
            actioncore)
        for role in roles:
            sense = None
            for dbr in finaldb.query('{}(?w, {}) ^ has_sense(?w, ?s)'.format(
                    role, actioncore)):
                sense = dbr['?s']
                result.append(
                    {'source': {'name': actioncore, 'text': ''},
                     'target': {'name': sense,
                                'text': wn.synset(sense).definition},
                     'value': role,
                     'arcStyle': 'strokegreen'})

        actioncore_dict['action_roles'].append({'role_name': role, 'role_value': sense})
        actioncores.append(actioncore_dict)



    pracsession.actioncores = actioncores
    return result
