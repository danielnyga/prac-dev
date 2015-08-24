from pracweb.gui.app import pracApp, app
import os
from os.path import expanduser
import logging
from logging import FileHandler

def register_routes(pracapp=None):
    print 'Registering PRAC routes...'
    pracApp.app = app
    pracApp.app.config['PRAC_STATIC_PATH'] = os.path.join(pracApp.app.root_path, 'build')

    # settings for fileupload and logging
    home = expanduser("~")
    pracApp.app.config['ALLOWED_EXTENSIONS'] = set(['mln','db','pracmln','emln'])
    pracApp.app.config['UPLOAD_FOLDER'] = os.path.join(home, 'pracfiles')

    pracApp.app.config['LOG_FOLDER'] = os.path.join(pracApp.app.root_path, 'log')
    if not os.path.exists(os.path.join(pracApp.app.config['LOG_FOLDER'])):
       os.mkdir(os.path.join(pracApp.app.config['LOG_FOLDER']))
    pracApp.app.secret_key = 'so secret!'

    # separate logger for user statistics
    ulog = logging.getLogger('userstats')
    ulog.setLevel(logging.INFO)
    formatter = logging.Formatter("%(message)s,")
    filelogger = FileHandler(os.path.join(pracApp.app.config['LOG_FOLDER'], "userstats.json"))
    filelogger.setFormatter(formatter)
    ulog.addHandler(filelogger)

    from pracweb.gui.pages import pracSpec
    from pracweb.gui.pages import inference
    from pracweb.gui.pages import views
    from pracweb.gui.pages import utils
    from pracweb.gui.pages import fileupload
