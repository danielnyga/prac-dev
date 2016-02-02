import os
import tempfile
from pracweb.gui.app import pracApp


class Config(object):
    SECRET_KEY = 'so secret!'

    # settings for fileupload and logging
    ALLOWED_EXTENSIONS = {'mln', 'db', 'pracmln', 'emln'}
    PRAC_STATIC_PATH = os.path.join(pracApp.app.root_path, 'build')
    UPLOAD_FOLDER = tempfile.gettempdir()
    PRAC_ROOT_PATH = os.path.join(pracApp.app.root_path, '..', '..')


class DeploymentConfig(Config):
    DEBUG = False
    THREADED = False
    TESTING = False
    WTF_CSRF_ENABLED = True
    CSRF_ENABLED = True


class DevelopmentConfig(Config):
    DEBUG = True
    THREADED = True
    TESTING = False
    WTF_CSRF_ENABLED = False


class TestingConfig(Config):
    DEBUG = False
    THREADED = False
    TESTING = True
    WTF_CSRF_ENABLED = False
