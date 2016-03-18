import os
import imp
import time
import json
import shutil
from urlparse import urlparse
from flask import render_template, request, send_from_directory, url_for, \
    jsonify, session, redirect
from prac.core.base import ActionCore
from prac.core.wordnet import WordNet
from pracmln.mln.util import colorize
from pracmln.praclog import logger
from pracweb.gui.app import pracApp
from pracweb.gui.pages.routes import ulogger
from pracweb.gui.pages.utils import get_file_content, \
    update_mln_list, update_evidence_list, update_kb_list, PRAC_HOME, \
    INFMETHODS, convert, ensure_prac_session


log = logger(__name__)
ulog = ulogger()


@pracApp.app.route('/prac/test/')
def prac_test():
    return render_template('test.html', **locals())


@pracApp.app.route('/prac/static/<path:filename>')
def download_prac_static(filename):
    return send_from_directory(pracApp.app.config['PRAC_STATIC_PATH'],
                               filename)


@pracApp.app.route('/prac/doc/<path:filename>')
def download_prac_docs(filename):
    return send_from_directory(os.path.join(
        pracApp.app.config['PRAC_ROOT_PATH'],
        'doc'), filename)


@pracApp.app.route('/prac/')
def prac():
    return render_template('welcome.html', **locals())


@pracApp.app.route('/prac/home/')
def _prac():
    ensure_prac_session(session)
    time.sleep(2)
    # return render_template('prac.html', **locals()) //openEASE integration
    return redirect('/prac/pracweb')


@pracApp.app.route('/prac/pracweb', methods=['GET', 'POST'])
def pracweb():
    ensure_prac_session(session)
    return render_template('infer.html', **locals())


@pracApp.app.after_request
def remove_if_invalid(response):
    if "__invalidate__" in session:
        response.delete_cookie(pracApp.app.session_cookie_name)
        prac_session = pracApp.session_store[session]
        if prac_session is not None:
            log.info('removed session %s' % prac_session.id.encode('base-64'))
            pracApp.session_store.remove(session)
        session.clear()
    return response


@pracApp.app.route('/prac/_destroy_session', methods=['POST', 'OPTIONS'])
def destroy():
    prac_session = pracApp.session_store[session]
    if prac_session is None:
        return ''
    if os.path.exists(prac_session.tmpsessionfolder):
        log.info('removing temp folder %s' % prac_session.tmpsessionfolder)
        shutil.rmtree(prac_session.tmpsessionfolder)
    log.info('invalidating session %s' % prac_session.id.encode('base-64'))
    session["__invalidate__"] = True
    return prac_session.id.encode('base-64')


@pracApp.app.route('/prac/_get_wordnet_taxonomy', methods=['GET'])
def get_wn_tax():
    wn = WordNet()
    return wn.to_svg()


@pracApp.app.route('/prac/menu', methods=['POST'])
def prac_menu():
    menu_left = []

    selection = "Options"
    choices = [('pracweb', url_for('prac') + 'pracweb')]

    menu_right = [
        ('CHOICES', (selection, choices))
    ]

    return jsonify(menu_left=menu_left, menu_right=menu_right)


@pracApp.app.route('/prac/log')
def praclog():
    return praclog_('null')


@pracApp.app.route('/prac/log/<fname>')
def praclog_(fname):
    if os.path.isfile(os.path.join(pracApp.app.config['LOG_FOLDER'], fname)):
        return send_from_directory(pracApp.app.config['LOG_FOLDER'], fname)
    elif os.path.isfile(os.path.join(pracApp.app.config['LOG_FOLDER'],
                                     '{}.json'.format(fname))):
        return send_from_directory(pracApp.app.config['LOG_FOLDER'],
                                   '{}.json'.format(fname))
    else:
        return render_template('userstats.html', **locals())


@pracApp.app.route('/prac/_user_stats', methods=['POST'])
def user_stats():
    data = convert(json.loads(request.get_data()))
    ip = data['ip'] if data['ip'] is not None else request.remote_addr
    stats = {}

    logstr = ("Wrote log entry:\n"
              "IP:\t\t{ip}\n"
              "Country:\t{country}\n"
              "Continent:\t{continent}\n"
              "Subdivisions:\t{subdivisions}\n"
              "Timezone:\t{timezone}\n"
              "Location:\t{location}\n"
              "Access Date:\t{date}\n"
              "Access Time:\t{time}")

    try:
        imp.find_module('geoip')
        try:
            from geoip import geolite2
            geolite = geolite2.lookup(ip)
            stats.update(geolite.to_dict())
            stats['subdivisions'] = ', '.join(
                stats['subdivisions'])  # prettify for log
        except AttributeError:
            logstr = ("Wrote log entry:\n"
                      "IP:\t\t\t\t{ip}\n"
                      "Access Date:\t{date}\n"
                      "Access Time:\t{time}")
        except ValueError:
            log.error('Not a valid ip address: {}'.format(ip))
        except KeyError:
            logstr = ("Wrote log entry:\n"
                      "IP:\t\t\t\t{ip}\n"
                      "Country:\t\t{country}\n"
                      "Continent:\t\t{continent}\n"
                      "Timezone:\t\t{timezone}\n"
                      "Location:\t\t{location}\n"
                      "Access Date:\t{date}\n"
                      "Access Time:\t{time}")
    except ImportError:
        print colorize('geoip module was not found. Install by "sudo pip '
                       'install python-geoip python-geoip-geolite2" if you '
                       'want to request geoip information',
                       (None, 'yellow', True), True)
    finally:
        stats.update({'ip': ip, 'date': data['date'], 'time': data['time']})
        ulog.info(json.dumps(stats))
        log.info(logstr.format(**stats))
        return ''


# route for qooxdoo resources
@pracApp.app.route('/prac/resource/<path:filename>')
def resource_file(filename):
    return redirect('/prac/static/resource/{}'.format(filename))


@pracApp.app.route('/prac/_get_modules', methods=['GET'])
def get_modules():
    pracsession = ensure_prac_session(session)
    return jsonify(
        modules=[module for module in pracsession.prac.moduleManifestByName],
        methods=INFMETHODS)


@pracApp.app.route('/prac/_load_flow_chart', methods=['GET'])
def _load_flow_chart():
    filename = os.path.join(os.path.join(PRAC_HOME, 'etc'),
                            'prac-flowchart.svg')
    with open(filename, 'r') as svgFile:
        content = svgFile.readlines()
    return ''.join(content)


@pracApp.app.route('/prac/update_module', methods=['POST'])
def update_module():
    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    module = data.get('module')
    kblist = [kb[0] for kb in update_kb_list(pracsession.prac, module,
                                             pracsession.tmpsessionfolder)]
    mlnlist = [mln[0] for mln in update_mln_list(pracsession.prac, module,
                                                 pracsession.tmpsessionfolder)]
    evidencelist = [ev[0] for ev in
                    update_evidence_list(pracsession.prac, module,
                                         pracsession.tmpsessionfolder)]

    return jsonify({'value': module, 'kblist': kblist, 'mlnlist': mlnlist,
                    'evidencelist': evidencelist})


@pracApp.app.route('/prac/updateUploadedFiles', methods=['GET'])
def update_uploaded_files():
    pracsession = ensure_prac_session(session)
    mlnlist = [mln[0] for mln in update_mln_list(pracsession.prac, None,
                                                 pracsession.tmpsessionfolder)]
    evidencelist = [ev[0] for ev in update_evidence_list(pracsession.prac,
                                                         None,
                                                         pracsession.tmpsessionfolder)]
    ret_data = {'mlnlist': mlnlist, 'evidencelist': evidencelist}

    return jsonify(ret_data)


@pracApp.app.route('/prac/update_text', methods=['POST'])
def update_text():
    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    filename = data['fName']
    modulename = data['module']

    # look for file in upload folder
    for root, subFolders, files in os.walk(pracsession.tmpsessionfolder):
        if filename in files:
            text = get_file_content(root, filename)
            return jsonify({'text': text})

    # look for file in module path
    if modulename in pracsession.prac.moduleManifestByName:
        module_path = pracsession.prac.moduleManifestByName[modulename].module_path
        for root, subFolders, files in os.walk(module_path):
            if filename in files:
                text = get_file_content(root, filename)
                return jsonify({'text': text})

    return jsonify({'text': ''})


@pracApp.app.route('/prac/updateKB', methods=['GET'])
def update_kb():
    pracsession = ensure_prac_session(session)
    if not request.args.get(
            'module') in pracsession.prac.moduleManifestByName or not request.args.get(
            'kb'):
        return jsonify({})
    module = pracsession.prac.getModuleByName(request.args.get('module'))
    kb = module.load_pracmt(request.args.get('kb'))
    res = kb.query_params
    res['mln'] = kb.query_mln_str
    return jsonify(res)


@pracApp.app.route('/prac/_init', methods=['GET'])
def init_options():
    pracsession = ensure_prac_session(session)
    actioncores = sorted(ActionCore.readFromFile(os.path.join(PRAC_HOME,
                                                              'models',
                                                              'actioncores.yaml')).keys())
    return jsonify({"actioncores": actioncores,
                    "data": pracsession.prac.wordnet.get_all_synsets()})
