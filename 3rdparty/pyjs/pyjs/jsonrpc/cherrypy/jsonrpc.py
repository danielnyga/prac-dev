# JSONRPCService for the CherrPy webserver.
# Copyright (C) 2011 Rene Maurer <renemaur@gmail.com>
# See LICENSE for details.
#
# Time-stamp: <2011-10-10 01:03:52 rene>
#
# Example usage:
#
# import cherrypy
# from pyjs.jsonrpc.cherrypy.jsonrpc import JSONRPCService
#
# class EchoService(JSONRPCService):
#
#     def __init__(self, defaultPage):
#         JSONRPCService.__init__(self, defaultPage)
#         self.add_method(self.echo.__name__, self.echo)    
#
#     def echo(self, msg):
#         return msg
#
# if __name__ == "__main__":
#     service =  EchoService(defaultPage='index.html')
#     configs = {'server.socket_port':8000, 'log.screen':True}
#     cherrypy.quickstart(service, '/', {'global':configs})

import os
import cherrypy
from cherrypy import log
from pyjs.jsonrpc import JSONRPCServiceBase

class JSONRPCService(JSONRPCServiceBase):

    def __init__(self, defaultPage=''):
        JSONRPCServiceBase.__init__(self)
        self.defaultPage = defaultPage

    '''
    Handle static pages.
    You may specify a default page for an empty request
    (e.g. http://localhost:8080).
    '''
    @cherrypy.expose
    def default(self, *args):
        try:
            filename = os.path.join(os.getcwd(), args[0])
        except:
            filename = os.path.join(os.getcwd(), self.defaultPage)
        if os.path.isfile(filename):
            log(os.linesep + '>>> Serve: %s' % filename)
            f = file(filename)
            s = f.read()
            f.close()
            return s
        log(os.linesep + '>>> Does not exist: %s' % filename)

    '''
    Handle jsonrpc requests.
    Note, that your proxy url *must* begin with 'services':
    E.g. pyjamas.JSONService.JSONProxy__init__('services', [method1, method2])
    '''
    @cherrypy.expose
    def services(self, *args):
        data = cherrypy.request.body.read()
        log(os.linesep + '>>> %s' % data)
        return self.process(data)
