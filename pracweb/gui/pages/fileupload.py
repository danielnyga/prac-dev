import os
from flask import request, send_from_directory, jsonify, redirect
from werkzeug import secure_filename
from pracweb.gui.pages.utils import FILEDIRS, init_file_storage
from pracweb.gui.app import pracApp


@pracApp.app.route('/prac/uploads/<filedir>/<filename>')
def uploaded_file(filedir, filename):
    if not 'UPLOAD_FOLDER' in pracApp.app.config:
        init_file_storage()
    return send_from_directory(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], filedir), filename)


@pracApp.app.route('/prac/resource/<path:filename>')
def resource_file(filename):
    return redirect('/prac/static/resource/{}'.format(filename)) 


def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1] in pracApp.app.config['ALLOWED_EXTENSIONS']


def upload(req):
    for f in req.files:
        tmpfile = req.files[f]
        if tmpfile and allowed_file(tmpfile.filename):
            filename = secure_filename(tmpfile.filename)
            fpath = os.path.join(pracApp.app.config['UPLOAD_FOLDER'], FILEDIRS.get(filename.rsplit('.', 1)[1], 'misc'))
            if not os.path.exists(fpath):
                os.mkdir(fpath)
            tmpfile.save(os.path.join(fpath, filename))


@pracApp.app.route('/prac/saveMLN/', methods=['POST'])
def save_mln():
    if not 'UPLOAD_FOLDER' in pracApp.app.config:
        init_file_storage()
    data = request.get_json()
    mlnpath = os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'mln')
    content = str(data['content'])
    fname = str(data['fName'])
    if '.' in fname and fname.rsplit('.', 1)[1] == 'mln':
        fullfilename = os.path.join(mlnpath, fname)
    else:
        fullfilename = os.path.join(mlnpath, "{}.mln".format(fname.rsplit('.', 1)[0]))

    if not os.path.exists(mlnpath):
        os.mkdir(mlnpath)
    with open(fullfilename,'w') as f:
        f.write(content)

    return jsonify({'path': fullfilename})
