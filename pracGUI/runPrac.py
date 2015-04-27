from pracWEB.pracinit import pracApp

def init_app(app):

    from pracWEB.pages.routes import register_routes
    # Load all views.py files to register @app.routes() with Flask
    register_routes()
    
    # Initialize app config settings
    pracApp.app.config['WTF_CSRF_ENABLED'] = False # Disable CSRF checks while testing

    return app


init_app(pracApp.app)


if __name__ == '__main__':
    pracApp.app.run(host='0.0.0.0', debug=True, threaded=True)
