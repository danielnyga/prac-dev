import pracmln
from pracweb.gui.app import pracApp
import os
from werkzeug.serving import run_simple


log = pracmln.praclog.logger(__name__)

def init_app(app):

    from gui.pages.routes import register_routes
    # Load all views.py files to register @app.routes() with Flask
    register_routes()
    
    # Initialize app config settings
    pracApp.app.config['WTF_CSRF_ENABLED'] = False # Disable CSRF checks while testing

    return app


init_app(pracApp.app)


if __name__ == '__main__':
    if 'PRAC_SERVER' in os.environ and os.environ['PRAC_SERVER'] == 'true':
        log.info('Running PRACWEB in server mode')
        certpath = os.path.dirname(os.path.realpath(__file__))
        context = (os.path.join(certpath, 'default.crt'), os.path.join(certpath, 'default.key'))
        run_simple('0.0.0.0', 5001, pracApp.app, ssl_context=context)
    else:
        log.info('Running PRACWEB in development mode')
        pracApp.app.run(host='0.0.0.0', port=5001, debug=True, threaded=True)

