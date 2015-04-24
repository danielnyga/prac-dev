from pracFlaskApp.pracinit import pracApp

from flask import render_template, request, send_from_directory, url_for, jsonify

# from webrob.pages.routes import ensure_prac_started
from pracFlaskApp.pages.learning import learn, PRACLearningForm
from pracFlaskApp.pages.inference import infer, PRACInferenceForm
from pracFlaskApp.pages.fileupload import upload, saveMLN
from pracFlaskApp.pages.utils import initFileStorage

import json

from urlparse import urlparse
import os

@pracApp.app.route('/prac/test/')
def test():
    return render_template('test.html')

@pracApp.app.route('/prac/static/<path:filename>')
def download_static(filename):
    return send_from_directory(pracApp.app.config['PRAC_STATIC_PATH'], filename)
    # return send_from_directory(os.path.join(pracApp.app.root_path, "static"), filename)

@pracApp.app.route('/prac/')
def prac():
    
    error=""
    host_url = urlparse(request.host_url).hostname
    container_name = ''

    return render_template('prac.html', **locals())


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
    form = PRACLearningForm(csrf_enabled=False)
    form.updateChoices()
    result = {'res': ''}

    if request.method == 'POST' and form.validate_on_submit():
        if 'uploadMLNFile' in request.form or 'uploadDBFile' in request.form:
            upload(request)
        elif 'saveMLNFile' in request.form:
            saveMLN(request)
        elif 'submit' in request.form:
            data = request.form
            files = request.files
            result['res'] = learn(data, {str(files[x].name) : str(files[x].filename) for x in files})
            return render_template('learn.html', **locals())

    return render_template('learn.html', **locals())


@pracApp.app.route('/prac/pracinfer', methods=['GET', 'POST'])
def pracinfer():
    if not 'UPLOAD_FOLDER' in pracApp.app.config:
        initFileStorage()
    form = PRACInferenceForm(csrf_enabled=False)
    form.updateChoices()
    result = {'res': ''}

    if request.method == 'POST' and form.validate_on_submit():
        if 'uploadMLNFile' in request.form:
            upload(request)
        elif 'submit' in request.form:
            data = request.form
            files = request.files
            result['steps'] = infer(data, {str(files[x].name) : str(files[x].filename) for x in files})
            return render_template('infer.html', **locals())

            # TODO: visualize
            # return render_template('result.html', form=form)
            # return result
    return render_template('infer.html', **locals())
