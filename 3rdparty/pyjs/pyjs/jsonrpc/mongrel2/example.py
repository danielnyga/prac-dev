"""
  Copyright (c) 2006 Jan-Klaas Kollhof

  This file is part of jsonrpc.

  jsonrpc is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation; either version 2.1 of the License, or
  (at your option) any later version.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this software; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
"""

#from pyjs.jsonrpc.mongrel2 import Mongrel2JSONRPCService, jsonremote
from __init__ import Mongrel2JSONRPCService, jsonremote

from mongrel2 import handler

sender_id = "82209006-86FF-4982-B5EA-D1E29E55D481"

conn = handler.Connection(sender_id, "tcp://127.0.0.1:9997",
                         "tcp://127.0.0.1:9996")
mongservice = Mongrel2JSONRPCService(conn)

@jsonremote(mongservice)
def echo(request, data):
    return data


while True:
    print "WAITING FOR REQUEST"
    mongservice() # handles one request
    
