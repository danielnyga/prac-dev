import BaseHTTPServer
import SocketServer
import os
import argparse
import logging
import tornado.netutil
import tornado.process
from werkzeug.serving import run_simple
from pracmln import praclog
from pracweb.gui.app import pracApp
from tornado.wsgi import WSGIContainer
from tornado.httpserver import HTTPServer
from tornado.ioloop import IOLoop

log = praclog.logger(__name__)


def init_app(app):

    from gui.pages.routes import register_routes
    # Load all views.py files to register @app.routes() with Flask
    register_routes()
    return app

init_app(pracApp.app)


if __name__ == '__main__':
    logging.getLogger().setLevel(logging.DEBUG)

    parser = argparse.ArgumentParser(description='pracweb')
    parser.add_argument('--gz-acquisition',
                        help='add option to acquire data for instruction in simulator',
                        action="store_true")
    parser.add_argument('--gz-simulation',
                        help='add option to simulate instruction execution in browser',
                        action="store_true")
    parser.add_argument('--instruction',
                        help='initialize browser with this instruction',
                        type=str)
    args = parser.parse_args()

    # store command line options
    pracApp.app.config['gz_acquisition'] = args.gz_acquisition
    pracApp.app.config['gz_simulation'] = args.gz_simulation
    pracApp.app.config['instruction'] = args.instruction 

    if 'PRAC_SERVER' in os.environ and os.environ['PRAC_SERVER'] == 'deploy':
        log.debug('Running PRACWEB in server mode')

        # load config
        pracApp.app.config.from_object('configmodule.DeploymentConfig')

        pracApp.app.run(host='0.0.0.0',
                        port=5001,
                        threaded=True)
    elif 'PRAC_SERVER' in os.environ and os.environ['PRAC_SERVER'] == 'testing':
        log.debug('Running PRACWEB in testing mode')

        # load config
        pracApp.app.config.from_object('configmodule.TestingConfig')

        pracApp.app.run(host='0.0.0.0',
                        threaded=True,
                        port=5001)
    elif 'PRAC_SERVER' in os.environ and os.environ['PRAC_SERVER'] == 'old':
        log.debug('Running PRACWEB in server mode')

        # load config
        pracApp.app.config.from_object('configmodule.Config')

        certpath = os.path.dirname(os.path.realpath(__file__))
        context = (os.path.join(certpath, 'default.crt'), os.path.join(certpath, 'default.key'))
        run_simple('0.0.0.0',
                   5001,
                   pracApp.app,
                   threaded=True,
                   ssl_context=context)
    else:
        log.debug('Running PRACWEB in development mode')

        # load config
        pracApp.app.config.from_object('configmodule.DevelopmentConfig')

        pracApp.app.run(host='0.0.0.0',
                        port=5001,
                        threaded=True)
