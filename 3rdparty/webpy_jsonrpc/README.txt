this example shows how you can use web.py with pyjamas jsonrpc example.

just run it: python code.py and browse to http://127.0.0.1:8080/
as you can see from code.py, only the reverse and echo functions are
supported: add your own implementations of uppercase and lowercase
if you wish.

to prepare your "own" version of the pyjamas example JSONRPCExample.py
rather than use the pre-compiled one, included, here, simply
modify the last few lines, remove the target "/cgi-bin/EchoService.py"
replace it with "/chat/" - to match with urls (see above) - and
rebuild (./build.sh).  copy the contents of the examples
jsonrpc/output directory into static/output, and you're good to go.


