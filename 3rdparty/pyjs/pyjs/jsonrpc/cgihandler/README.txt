Example usage.  very very simple.  yes, really, it's this simple.

#!/usr/bin/env python

from pyjs.jsonrpc.cgihandler import jsonremote, CGIJSONRPCService
svc = CGIJSONRPCService()

# add functions here

@jsonremote(svc)
def echo(msg):
    return msg

# run the cgi handler
if __name__ == '__main__':
    svc()

