# please could someone add an example usage or link to example usage,
# this is very very unclear as to what is going on, even though it's
# incredibly short.

from pyjs.jsonrpc import JSONRPCServiceBase, jsonremote

class JSONRPCService(JSONRPCServiceBase):

    def serve(self):
        return self.process(request.body.read())

    def __call__(self,func):
        self.methods[func.__name__] = func
        return func

