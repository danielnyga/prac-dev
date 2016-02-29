import os
import logging
from logging import FileHandler
from pracweb.gui.app import pracApp, app


def ulogger(): return logging.getLogger('userstats')


def register_routes():
    print 'Registering PRAC routes...'
    pracApp.app = app

    pracApp.app.config['LOG_FOLDER'] = os.path.join(pracApp.app.root_path, 'log')
    if not os.path.exists(os.path.join(pracApp.app.config['LOG_FOLDER'])):
        os.mkdir(os.path.join(pracApp.app.config['LOG_FOLDER']))

    # separate logger for user statistics
    root_logger = logging.getLogger('userstats')
    handler = FileHandler(os.path.join(
        pracApp.app.config['LOG_FOLDER'], "userstats.json"))
    formatter = logging.Formatter("%(message)s,")
    handler.setFormatter(formatter)
    root_logger.addHandler(handler)

    from pracweb.gui.pages import pracSpec
    from pracweb.gui.pages import inference
    from pracweb.gui.pages import views
    from pracweb.gui.pages import utils
    from pracweb.gui.pages import cram
