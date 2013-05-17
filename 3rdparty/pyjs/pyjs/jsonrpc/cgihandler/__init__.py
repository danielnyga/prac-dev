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

from pyjs.jsonrpc import JSONRPCServiceBase, jsonremote
import sys, os
import Cookie

def read_data():
    try:
        contLen=int(os.environ['CONTENT_LENGTH'])
        data = sys.stdin.read(contLen)
    except:
        data = ""
    return data

def write_data(data, cookies):
    if not isinstance(data, list):
        data = [data]
    data = "\n".join(data)
    response = "Content-Type: text/plain\n"
    if cookies:
        response += cookies+"\n"
    response += "Content-Length: %d\n\n" % len(data)
    response += data
    
    #on windows all \n are converted to \r\n if stdout is a terminal and 
    # is not set to binary mode :(
    #this will then cause an incorrect Content-length.
    #I have only experienced this problem with apache on Win so far.
    if sys.platform == "win32":
        import  msvcrt
        msvcrt.setmode(sys.stdout.fileno(), os.O_BINARY)
    #put out the response
    sys.stdout.write(response)

class CGIJSONRPCService(JSONRPCServiceBase):
    def set_cookies(self, c):
        self._cookies = c

    def __call__(self):
        self._cookies = Cookie.SmartCookie()
        self._cookies.load(os.environ.get('HTTP_COOKIE', ''))
        d = read_data() # TODO: handle partial data
        write_data(self.process(d), self._cookies.output())

