from webrob.pages.routes import register_routes
from webrob.pracinit import pracApp

def init_app(app):

    # Load all views.py files to register @app.routes() with Flask
    register_routes(pApp='PracFlask')
    
    # Initialize app config settings
    pracApp.app.config['WTF_CSRF_ENABLED'] = False # Disable CSRF checks while testing

    return app

