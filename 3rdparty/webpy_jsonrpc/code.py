from network import JSONRPCService, jsonremote
import web
import os

urls = (
    '/chat/', 'chatcls',
    '/chat',  'chatcls',
    '/(.*)', 'static' # static content
    )

# to wrap static content like this is possibly a bit of an unnecessary hack,
# but it makes me feel better.  as i am unfamiliar with web.py, i don't
# entirely know how it supports static content, so feel more comfortable
# with doing it manually, here, with this "static" class.

class static:
    def GET(self, name):

        if name == '':
            name = 'JSONRPCExample.html'

        ext = name.split(".")[-1] 

        cType = {
            "js":"application/x-javascript",
            "txt":"text/plain",
            "css":"text/css",
            "html":"text/html",
            "png":"images/png",
            "jpg":"image/jpeg",
            "gif":"image/gif",
            "ico":"image/x-icon"            }

        if name in os.listdir('static/output'): 
            web.header("Content-Type", cType[ext]) 
            print open('static/output/%s'%name,"rb").read() 
        else:
            web.notfound()

# a "wrapper" class around the jsonrpc service "chatservice"
class chatcls:
    def POST(self):
        print chatservice(web.webapi.data())

chatservice = JSONRPCService()

# the two demo functions

@jsonremote(chatservice)
def echo(request, user_name):
    web.debug(repr(request))
    return "hello world %s" % user_name

@jsonremote(chatservice)
def reverse(request, user_name):
    return "hello world %s" % user_name[::-1]

# start me!
if __name__ == "__main__":
    web.run(urls, globals())

