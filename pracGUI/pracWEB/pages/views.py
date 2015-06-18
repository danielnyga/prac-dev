from pracWEB.pracinit import pracApp

from flask import render_template, request, send_from_directory, url_for, jsonify, session, redirect

from pracWEB.pages.learning import learn 
from pracWEB.pages.fileupload import upload, saveMLN
from pracWEB.pages.utils import initFileStorage

import json

from urlparse import urlparse
import time
import logging
from prac.core import PRAC
from prac.wordnet import WordNet
from pracWEB.pages.utils import ensure_prac_session

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
    return response

@pracApp.app.route('/prac/_destroy_session', methods=['POST', 'OPTIONS'])
def destroy():
    log = logging.getLogger(__name__)
    prac_session = pracApp.session_store[session]
    if prac_session is None: return ''
    log.info('invalidating session %s' % prac_session.id.encode('base-64'))
    session["__invalidate__"] = True
    return ''

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


@pracApp.app.route('/prac/praclearn', methods=['GET', 'POST'])
def praclearn():
    if not 'UPLOAD_FOLDER' in pracApp.app.config:
        initFileStorage()
#     form = PRACLearningForm(csrf_enabled=False)
#     form.updateChoices()
#     result = {'res': ''}
# 
#     if request.method == 'POST' and form.validate_on_submit():
#         if 'uploadMLNFile' in request.form or 'uploadDBFile' in request.form:
#             upload(request)
#         elif 'saveMLNFile' in request.form:
#             saveMLN(request)
#         elif 'submit' in request.form:
#             data = request.form
#             files = request.files
#             result['res'] = learn(data, {str(files[x].name) : str(files[x].filename) for x in files})
#             return render_template('learn.html', **locals())

    return render_template('learn.html', **locals())


@pracApp.app.route('/prac/pracinfer', methods=['GET', 'POST'])
def pracinfer():
    ensure_prac_session(session)
#     if not 'UPLOAD_FOLDER' in pracApp.app.config:
#         initFileStorage()
#     form = PRACInferenceForm(csrf_enabled=False)
#     form.updateChoices()
#     result = {'res': ''}
# 
#     if request.method == 'POST' and form.validate_on_submit():
#         if 'uploadMLNFile' in request.form:
#             upload(request)
#         elif 'submit' in request.form:
#             data = request.form
#             files = request.files
#             result['steps'] = infer(data, {str(files[x].name) : str(files[x].filename) for x in files})
#             return render_template('infer.html', **locals())

            # TODO: visualize
            # return render_template('result.html', form=form)
            # return result
    return render_template('infer.html', **locals())
