from pracWEB.pracinit import pracApp
import os

def register_routes():
    print 'register routes'
    from pracWEB.app import app
    pracApp.app = app
    print pracApp.app
    pracApp.app.config['PRAC_STATIC_PATH'] = os.path.join(pracApp.app.root_path, 'build')
    pracApp.app.secret_key = 'so secret!'
    from pracWEB.pages import pracSpec

    from pracWEB.pages import views
    from pracWEB.pages import utils
