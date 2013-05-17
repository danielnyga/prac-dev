# JSONRPCService and jsonremote are used in combination to drastically
# simplify the provision of JSONRPC services.  use as follows:
#
# jsonservice = JSONRPCService()
#
# @jsonremote(jsonservice)
# def test(request, echo_param):
#     return "echoing the param back: %s" % echo_param
#
# dump jsonservice into urlpatterns:
#  (r'^service1/$', 'djangoapp.views.jsonservice'),

from pyjs.jsonrpc import JSONRPCServiceBase, jsonremote

class JSONRPCService(JSONRPCServiceBase):
    
    def __call__(self, request, extra=None):
        return self.process(request.raw_post_data)

# just for convenience and for fun...
from jsonformprocessor import *

