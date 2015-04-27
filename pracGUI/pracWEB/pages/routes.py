from pracWEB.pracinit import pracApp
import os

def register_routes():

    from pracWEB.app import app
    pracApp.app = app
    pracApp.app.config['PRAC_STATIC_PATH'] = os.path.join(pracApp.app.root_path, 'build')
    from pracWEB.pages import pracSpec

    from pracWEB.pages import views
    from pracWEB.pages import utils
