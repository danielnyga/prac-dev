import sys
import traceback

# some dog's dinner random ways to get a json library from somewhere... 
try:
    import json
except ImportError, e:
    try:
        import gluon.contrib.simplejson as json
    except ImportError, e:
        import simplejson as json
 
# this base class, use it to call self.process
class JSONRPCServiceBase:

    def __init__(self):
        self.methods={}

    def response(self, mid, result):
        return json.dumps({'version': '1.1', 'id':mid,
                                 'result':result, 'error':None})
    def error(self, mid, code, message):
        return json.dumps({'id': mid,
                                 'version': '1.1',
                                 'error': {'name': 'JSONRPCError',
                                           'code': code,
                                           'message': message
                                           }
                                     })

    def add_method(self, name, method):
        self.methods[name] = method

    def process(self, data):
        data = json.loads(data)
        msgid, method, params = data["id"], data["method"], data["params"]
        if method in self.methods:
            try:
                result =self.methods[method](*params)
                return self.response(msgid, result)
            except BaseException:
                etype, eval, etb = sys.exc_info()
                tb = traceback.format_tb(etb)
                return self.error(msgid, 100, '%s: %s\n%s' % \
                        (etype.__name__, eval, '\n'.join(tb)))
            except:
                etype, eval, etb = sys.exc_info()
                tb = traceback.format_tb(etb)
                return self.error(msgid, 100, '%s: %s\n%s' % \
                        (etype.__name__, eval, '\n'.join(tb)))
        else:
            return self.error(msgid, 100, 'method "%s" does not exist' % method)

    def listmethods(self):
        return self.methods.keys() 

def jsonremote(service):
    """Make JSONRPCService a decorator so that you can write :
    
    import JSONRPCService (note: this must derive from JSONRPCServiceBase!)
    chatservice = JSONRPCService()

    @jsonremote(chatservice)
    def login(request, user_name):
        (...)
    """
    def remotify(func):
        if isinstance(service, JSONRPCServiceBase):
            service.add_method(func.__name__, func)
        else:
            emsg = 'Service "%s" not found' % str(service.__name__)
            raise NotImplementedError, emsg
        return func
    return remotify



