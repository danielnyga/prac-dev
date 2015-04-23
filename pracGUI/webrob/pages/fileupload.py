import os
from flask import request, send_from_directory, jsonify
from werkzeug import secure_filename
from webrob.pracinit import pracApp
from webrob.pages.utils import FILEDIRS, initFileStorage

@pracApp.app.route('/prac/uploads/<filedir>/<filename>')
def uploaded_file(filedir, filename):
    if not 'UPLOAD_FOLDER' in pracApp.app.config:
        initFileStorage()
    return send_from_directory(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], filedir), filename)


def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1] in pracApp.app.config['ALLOWED_EXTENSIONS']


def upload(request):
    for f in request.files:
        file = request.files[f]
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            fpath = os.path.join(pracApp.app.config['UPLOAD_FOLDER'], FILEDIRS.get(filename.rsplit('.', 1)[1], 'misc'))
            if not os.path.exists(fpath):
                os.mkdir(fpath)
            file.save(os.path.join(fpath, filename))


@pracApp.app.route('/prac/saveMLN/', methods=['POST'])
def saveMLN():
    if not 'UPLOAD_FOLDER' in pracApp.app.config:
        initFileStorage()
    data = request.get_json()
    MLNPATH = os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'mln')
    content = str(data['content'])
    fName = str(data['fName'])
    if '.' in fName and fName.rsplit('.', 1)[1] == 'mln':
        fullFileName = os.path.join(MLNPATH, fName)
    else:
        fullFileName = os.path.join(MLNPATH, "{}.mln".format(fName.rsplit('.', 1)[0]))

    if not os.path.exists(MLNPATH):
        os.mkdir(MLNPATH)    
    with open(fullFileName,'w') as f:
        f.write(content)

    return jsonify({'path': fullFileName})

