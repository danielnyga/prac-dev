Michele Comitini michele.comitini@gmail.com via pyjs.org to pyjamasdev
    
show details 3:09 PM (1 hour ago)
    
Hi Luke,


I use web2py as JSONRPC server.  No need to do anything: it works out
of the box.
Just add @service.jsonrpc annotation.

@service.jsonrpc
def my_service(a,b,c):
  .
  .
  .
  return dict( . . . )

More at http://web2py.com/book/default/chapter/09#JSONRPC-and-Pyjamas

