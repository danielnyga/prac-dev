from pracweb.gui.app import pracApp, app
import os
from os.path import expanduser
import logging
from logging import FileHandler


def ulogger(name): return logging.getLogger('userstats')


def register_routes(pracapp=None):
    print 'Registering PRAC routes...'
    pracApp.app = app
    pracApp.app.config['PRAC_STATIC_PATH'] = os.path.join(pracApp.app.root_path, 'build')

    # settings for fileupload and logging
    home = expanduser("~")
    pracApp.app.config['ALLOWED_EXTENSIONS'] = {'mln', 'db', 'pracmln', 'emln'}
    pracApp.app.config['UPLOAD_FOLDER'] = os.path.join(home, 'pracfiles')

    pracApp.app.config['LOG_FOLDER'] = os.path.join(pracApp.app.root_path, 'log')
    if not os.path.exists(os.path.join(pracApp.app.config['LOG_FOLDER'])):
        os.mkdir(os.path.join(pracApp.app.config['LOG_FOLDER']))
    pracApp.app.secret_key = 'so secret!'

    # separate logger for user statistics
    root_logger = logging.getLogger('userstats')
    handler = FileHandler(os.path.join(pracApp.app.config['LOG_FOLDER'], "userstats.json"))
    formatter = logging.Formatter("%(message)s,")
    handler.setFormatter(formatter)
    root_logger.addHandler(handler)

    from pracweb.gui.pages import pracSpec
    from pracweb.gui.pages import inference
    from pracweb.gui.pages import views
    from pracweb.gui.pages import utils
    from pracweb.gui.pages import fileupload
