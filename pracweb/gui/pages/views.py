from pracweb.gui.app import pracApp
import json
import os

from flask import render_template, request, send_from_directory, url_for, jsonify, session, redirect
from geoip import geolite2
from pracweb.gui.pages.utils import initFileStorage, getFileContent, \
    updateMLNList, updateEvidenceList, updateKBList, PRAC_HOME, INFMETHODS, \
    convert

from urlparse import urlparse
import time
import logging
from prac.core.wordnet import WordNet
from pracweb.gui.pages.utils import ensure_prac_session

@pracApp.app.route('/prac/test/')
def test():
    return render_template('test.html')

@pracApp.app.route('/prac/static/<path:filename>')
def download_static(filename):
    return send_from_directory(pracApp.app.config['PRAC_STATIC_PATH'], filename)

@pracApp.app.route('/prac/')
def prac():
    return render_template('welcome.html', **locals())

@pracApp.app.route('/prac/home/')
def _prac():
    log = logging.getLogger(__name__)
    error = ''
    host_url = urlparse(request.host_url).hostname
    container_name = ''
    ensure_prac_session(session)
    time.sleep(2)
    # return render_template('prac.html', **locals()) // for openEASE integration
    return redirect('/prac/pracinfer') 

@pracApp.app.after_request
def remove_if_invalid(response):
    log = logging.getLogger(__name__)
    if "__invalidate__" in session:
        response.delete_cookie(pracApp.app.session_cookie_name)
        prac_session = pracApp.session_store[session]
        if prac_session is not None:
            log.info('removed PRAC session %s' % prac_session.id.encode('base-64'))
            pracApp.session_store.remove(session)
        session.clear()
    return response

@pracApp.app.route('/prac/_destroy_session', methods=['POST', 'OPTIONS'])
def destroy():
    log = logging.getLogger(__name__)
    prac_session = pracApp.session_store[session]
    if prac_session is None: return ''
    log.info('invalidating session %s' % prac_session.id.encode('base-64'))
    session["__invalidate__"] = True
    return prac_session.id.encode('base-64')

@pracApp.app.route('/prac/_get_wordnet_taxonomy', methods=['GET'])
def get_wn_tax():
    wn = WordNet()
    return wn.to_svg()

@pracApp.app.route('/prac/menu', methods=['POST'])
def menu():
    menu_left = []
    
    selection = "Options"
    choices =  [('PracLEARN', url_for('prac')+'praclearn'),('PracINFER', url_for('prac')+'pracinfer')]
    
    menu_right = [
        ('CHOICES', (selection, choices))
    ]
    
    return jsonify(menu_left=menu_left, menu_right=menu_right)


@pracApp.app.route('/prac/log')
def praclog():
    return praclog_('null')

@pracApp.app.route('/prac/log/<filename>')
def praclog_(filename):
    if os.path.isfile(os.path.join(pracApp.app.config['LOG_FOLDER'], filename)):
        return send_from_directory(pracApp.app.config['LOG_FOLDER'], filename)
    elif os.path.isfile(os.path.join(pracApp.app.config['LOG_FOLDER'],
                                     '{}.json'.format(filename))):
        return send_from_directory(pracApp.app.config['LOG_FOLDER'], '{}.json'.format(filename))
    else:
        return render_template('userstats.html', **locals())

@pracApp.app.route('/prac/_user_stats', methods=['POST'])
def user_stats():
    ulog = logging.getLogger('userstats')

    data = convert(json.loads(request.get_data()))
    print 'user_stats', data
    print 'ip from request', request.remote_addr
    ip = data['ip'] if data['ip'] is not None else request.remote_addr
    stats = {}

    logstr = ("Wrote log entry:\n"
                    "IP:\t\t\t\t{ip}\n"
                    "Country:\t\t{country}\n"
                    "Continent:\t\t{continent}\n"
                    "Subdivisions:\t{subdivisions}\n"
                    "Timezone:\t\t{timezone}\n"
                    "Location:\t\t{location}\n"
                    "Access Date:\t{date}\n"
                    "Access Time:\t{time}")

    try:
        geolite = geolite2.lookup(ip)
        stats.update(geolite.to_dict())
        stats['subdivisions'] = ', '.join(stats['subdivisions']) # prettify for log
    except AttributeError:
        print 'using reduced logging string'
        logstr = ("Wrote log entry:\n"
                    "IP:\t\t\t\t{ip}\n"
                    "Access Date:\t{date}\n"
                    "Access Time:\t{time}")
    except ValueError:
        print 'Not a valid ip address:', ip
    except KeyError:
        logstr = ("Wrote log entry:\n"
                    "IP:\t\t\t\t{ip}\n"
                    "Country:\t\t{country}\n"
                    "Continent:\t\t{continent}\n"
                    "Timezone:\t\t{timezone}\n"
                    "Location:\t\t{location}\n"
                    "Access Date:\t{date}\n"
                    "Access Time:\t{time}")
    finally:
        stats.update({'ip': ip, 'date':data['date'], 'time':data['time']})
        ulog.info(json.dumps(stats))
        print logstr.format(**stats)
        return ''

@pracApp.app.route('/prac/_get_modules', methods=['GET'])
def get_modules():
    pracsession = ensure_prac_session(session)
    print 'infmethods', INFMETHODS
    return jsonify( modules=[module for module in pracsession.prac.moduleManifestByName], methods=INFMETHODS)

@pracApp.app.route('/prac/_load_flow_chart', methods=['GET'])
def _load_flow_chart():
    filename = os.path.join(os.path.join(PRAC_HOME, 'etc'), 'prac-flowchart.svg')
    with open(filename, 'r') as svgFile:
        content = svgFile.readlines()
    return ''.join(content)

@pracApp.app.route('/prac/update_module', methods=['POST'])
def updateModule():
    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    module = data.get('module')
    kbList = [kb[0] for kb in updateKBList(pracsession.prac, module)]
    mlnList = [mln[0] for mln in updateMLNList(pracsession.prac, module)]
    evidenceList = [ev[0] for ev in updateEvidenceList(pracsession.prac, module)]

    return jsonify({'value': module,'kblist': kbList, 'mlnlist': mlnList, 'evidencelist': evidenceList})


@pracApp.app.route('/prac/updateUploadedFiles', methods=['GET'])
def updateUploadedFiles():
    mlnList = [mln[0] for mln in updateMLNList(None)]
    evidenceList = [ev[0] for ev in updateEvidenceList(None)]
    ret_data = {'mlnlist': mlnList, 'evidencelist': evidenceList}

    return jsonify(ret_data)


@pracApp.app.route('/prac/update_text', methods=['POST'])
def updateText():
    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    fileName = data['fName']
    moduleName = data['module']

    # look for file in upload folder
    for root, subFolders, files in os.walk(pracApp.app.config['UPLOAD_FOLDER']):
        if fileName in files:
            filePathModule = os.path.join(root, fileName)
            text = getFileContent(root, fileName)
            return jsonify({'text': text})

    # look for file in module path
    if moduleName in pracsession.prac.moduleManifestByName:
        module_path = pracsession.prac.moduleManifestByName[moduleName].module_path
        for root, subFolders, files in os.walk(module_path):
            if fileName in files:
                text = getFileContent(root, fileName)
                return jsonify({'text': text})

    return jsonify({'text': ''})


@pracApp.app.route('/prac/updateKB', methods=['GET'])
def updateKB():
    pracsession = ensure_prac_session(session)
    if not request.args.get('module') in pracsession.prac.moduleManifestByName or not request.args.get('kb'):
        return jsonify({})
    module = pracsession.prac.getModuleByName(request.args.get('module'))
    kb = module.load_pracmt(request.args.get('kb'))
    res = kb.query_params
    res['mln'] = kb.query_mln_str
    return jsonify(res)


@pracApp.app.route('/prac/praclearn', methods=['GET', 'POST'])
def praclearn():
    if not 'UPLOAD_FOLDER' in pracApp.app.config:
        initFileStorage()
    return render_template('learn.html', **locals())


@pracApp.app.route('/prac/pracinfer', methods=['GET', 'POST'])
def pracinfer():
    ensure_prac_session(session)
            # TODO: visualize
    return render_template('infer.html', **locals())
