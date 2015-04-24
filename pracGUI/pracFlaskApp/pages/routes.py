from pracFlaskApp.pracinit import pracApp
import os

def register_routes():

    from pracFlaskApp.app import app
    pracApp.app = app
    pracApp.app.config['PRAC_STATIC_PATH'] = os.path.join(pracApp.app.root_path, 'static')
    from pracFlaskApp.pages import pracSpec

    from pracFlaskApp.pages import views
    from pracFlaskApp.pages import utils
