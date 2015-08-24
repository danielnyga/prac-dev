from pracweb.gui.app import pracApp
import logging

def init_app(app):

    from gui.pages.routes import register_routes
    # Load all views.py files to register @app.routes() with Flask
    register_routes()
    
    # Initialize app config settings
    pracApp.app.config['WTF_CSRF_ENABLED'] = False # Disable CSRF checks while testing

    return app


init_app(pracApp.app)


if __name__ == '__main__':
    logging.getLogger().setLevel(logging.INFO)
    pracApp.app.run(host='0.0.0.0', port=5001, debug=True, threaded=True)