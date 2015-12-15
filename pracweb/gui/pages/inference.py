import traceback
from flask import request, jsonify
import sys
from StringIO import StringIO
from flask.globals import session
from threading import Thread
import json
import logging
from prac.core.inference import PRACInference
from prac.pracutils.ActioncoreDescriptionHandler import \
    ActioncoreDescriptionHandler
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


# @pracApp.app.route('/prac/_start_inference', methods=['POST'])
# def start_inference():
#     pracsession = ensure_prac_session(session)
#     pracsession.infbuffer = RequestBuffer()
#     t = Thread(target=_pracinfer_step, args=(pracsession))
#     t.start()
#     pracsession.infbuffer.waitformsg()
#     return jsonify(pracsession.infbuffer.content)
#
#
# @pracApp.app.route('/prac/_get_status', methods=['POST'])
# def getinfstatus():
#     pracsession = ensure_prac_session(session)
#     pracsession.infbuffer.waitformsg()
#     return jsonify(pracsession.infbuffer.content)

# initialize logger
stream = StringIO()
handler = logging.StreamHandler(stream)
sformatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(sformatter)
streamlog = logging.getLogger('streamlog')
streamlog.setLevel(logging.INFO)
streamlog.addHandler(handler)
streamlog.info('STARTING INFERENCE')



@pracApp.app.route('/prac/_pracinfer_step', methods=['POST', 'GET'])
def _pracinfer_step():
    pracsession = ensure_prac_session(session)
    prac = pracsession.prac

    logmsg = ''
    result = []
    settings = {}
    finish = False
    msg = ''
    try:
        if request.method == 'POST' or (hasattr(pracsession, 'infer') and pracsession.infer.next_module() is not None):
            # any step except first and last
            if hasattr(pracsession, 'infer') and pracsession.infer.next_module() is not None:
                # no executable plan has been found so far
                if pracsession.infer.next_module() is not None:
                    module = prac.getModuleByName(pracsession.infer.next_module())
                    streamlog.info('Running Module {}'.format(module.name))
                    prac.run(pracsession.infer, module)
                    if module.name == 'senses_and_roles':
                        pracsession.sar_step = pracsession.infer.inference_steps[-1]

            # first inference step (nl parsing)
            else:
                data = json.loads(request.get_data())
                pracsession.count = 1
                if data['acatontology']:
                    prac.wordnet = awn
                else:
                    prac.wordnet = wn
                streamlog.info('starting new PRAC inference on "{}"'.format(data['sentence']))
                pracsession.prac = prac
                infer = PRACInference(prac, [data['sentence']])
                parser = prac.getModuleByName('nl_parsing')
                prac.run(infer, parser)
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
            streamlog.info('Finalizing result...')
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

    except:
        traceback.print_exc(file=stream)
        msg = 'Failed!\nCheck log for more information.'
    finally:
        handler.flush()
        streamlog.info('\n')
        value = stream.getvalue()
        if finish: stream.truncate(0)
        try:
            logmsg += u'\n%s' % unicode(value, 'utf-8')
        except UnicodeError:
            traceback.print_exc()
            logmsg += u'\n%s' % repr(value)

        return jsonify({'result': result,
                        'finish': finish,
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
        db.write(stream, color=False, bars=False)
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


def generate_final_links(pracsession):
    result = []
    finaldb = Database(pracsession.prac.mln)
    for step in pracsession.infer.inference_steps:
        for db in step.output_dbs:
            for atom, truth in db.evidence.iteritems():
                if truth == 0: continue
                _, predname, _ = pracsession.prac.mln.logic.parse_literal(
                    atom)
                if predname in ActioncoreDescriptionHandler.roles().union(
                        ['has_sense', 'action_core', 'achieved_by']):
                    finaldb << (atom, truth)

    # add all roles and word sense (is_a) links
    for res in finaldb.query(
            'action_core(?w, ?a) ^ has_sense(?w, ?s)'):
        actioncore = res['?a']
        sense = res['?s']
        result.append({'source': {'name': actioncore, 'text': ''},
                       'target': {'name': sense,
                                  'text': wn.synset(sense).definition},
                       'value': 'is_a', 'arcStyle': 'strokegreen'})

        roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(
            actioncore)
        for role in roles:
            for dbres in finaldb.query(
                            '%s(?w, %s) ^ has_sense(?w, ?s)' % (
                            role, actioncore)):
                sense = dbres['?s']
                result.append(
                    {'source': {'name': actioncore, 'text': ''},
                     'target': {'name': sense,
                                'text': wn.synset(sense).definition},
                     'value': role,
                     'arcStyle': 'strokegreen'})
    # add achieved_by links
    for finaldbres in finaldb.query('achieved_by(?a1, ?a2)'):
        a1 = finaldbres['?a1']
        actioncore = finaldbres['?a2']
        result.append({'source': {'name': a1, 'text': ''},
                       'target': {'name': actioncore, 'text': ''},
                       'value': 'achieved_by',
                       'arcStyle': 'strokegreen'})
        roles = ActioncoreDescriptionHandler.getRolesBasedOnActioncore(
            actioncore)
        for role in roles:
            for dbr in finaldb.query(
                            '%s(?w, %s) ^ has_sense(?w, ?s)' % (
                            role, actioncore)):
                sense = dbr['?s']
                result.append(
                    {'source': {'name': actioncore, 'text': ''},
                     'target': {'name': sense,
                                'text': wn.synset(sense).definition},
                     'value': role,
                     'arcStyle': 'strokegreen'})
    return result
