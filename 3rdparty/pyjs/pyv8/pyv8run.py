#!/usr/bin/env python

import PyV8

import os
import sys

from os.path import join, dirname, basename, abspath
from optparse import OptionParser

usage = """
  usage: %prog [options] <application module name or path>
"""

currentdir = abspath(dirname(__file__))
pyjspth = abspath(join(dirname(__file__), ".."))
sys.path = [(join(pyjspth, "pyjs", "src"))] + sys.path

import pyjs

pyjs.pyjspth = pyjspth
pyjs.path += [os.path.join(pyjspth, 'library'),
            os.path.join(pyjspth, 'addons'),
]


from pyjs import translator
from linker import PLATFORM, PyV8Linker, add_linker_options
from jsglobal import Global

class JSRuntimeError(Exception):
    def __init__(self, ctxt, exc):
        self.module = ctxt.locals['$pyjs']['track']['module']        
        self.lineno = ctxt.locals['$pyjs']['track']['lineno']
        self.traceback = self.get_traceback(ctxt.locals['$pyjs']['trackstack'])
        if ':' in exc.message and not ' ' in exc.message.split(':', 1)[0]:
            errortype, errortext = exc.message.split(':', 1)
            self.jserrortype = errortype
            self.jserrortext = errortext
        else:
            self.jserrortype = exc.__class__.__name__
            self.jserrortext = exc.message
    
    def __str__(self):
        return "{0.jserrortype}: {0.module}.py:{0.lineno} {0.jserrortext}".\
               format(self)
    
    def full(self):
        return ("{0.jserrortype}: {0.module}.py:{0.lineno}\n"
                "Traceback:\n{trace}\n"
                "{0.jserrortext}".format(self, trace=self.get_traceback_text()))
    
    def get_traceback(self, jstb):
        tb = []
        for x in jstb:
            tb.append(dict(module=x['module'], lineno=x['lineno']))
        return tb
    
    def get_traceback_text(self):
        trace = []
        for x in self.traceback:
            trace.append("  {0}.py:{1}".format(x['module'], x['lineno']))
        return "\n".join(trace)

def main():
    usage = """
    usage: %prog [ options ] [ -c command | module_name | script | - ] [ -- arguments ]
    """
    parser = OptionParser(usage = usage)
    translator.add_compile_options(parser)

    parser.add_option(
        "--dynamic",
        dest="unlinked_modules",
        action="append",
        help="regular expression for modules that will not be linked"
        "and thus loaded dynamically"
        )
    parser.add_option(
        "-c",
        dest="command",
        help="Python command to run")
    
    args = sys.argv[1:]
    app_args = []
    if '--' in args:
        idx = args.index('--')
        app_args = args[idx+1:]
        args = args[0:idx]
    # override the default because we want print
    parser.set_defaults(print_statements=True)
    add_linker_options(parser)
    options, args = parser.parse_args(args)
    IS_REPL = False
    if len(args) == 0 or args[0] == '-':
        IS_REPL = True
        modules = ['main']
    else:
        modules = args
    
    app_args[0:0] = [modules[0]]
    
    _modules = []
    for mod in modules:
        if mod.startswith('.') or mod.startswith('/') or mod.endswith('.py'):
            pyjs.path[0:0] = [os.path.dirname(os.path.abspath(mod))]
            _modules.append(os.path.basename(mod).split('.')[0])
        else:
            _modules.append(mod)
    modules = _modules
    
    pyjs.path[0:0] = [join(pyjs.pyjspth, 'stdlib')]
    pyjs.path.append(join(pyjs.pyjspth, 'pyjs', 'src'))
    
    for d in options.library_dirs:
        pyjs.path.append(os.path.abspath(d))

    #print "paths:", pyjs.path
    translator_arguments = translator.get_compile_options(options)
    linker = PyV8Linker(modules, output=options.output,
                        platforms=[PLATFORM],
                        path=pyjs.path,
                        compiler=translator.compiler,
                        translator_arguments=translator_arguments)
    linker()
    
    fp = open(linker.out_file_mod, 'r')
    txt = fp.read()
    fp.close()

    #PyV8.debugger.enabled = True
    
    # create a context with an explicit global
    g = Global(app_args, pyjs.path)
    ctxt = PyV8.JSContext(g)
    g.__context__ = ctxt
    # enter the context
    ctxt.enter()
    try:
        x = ctxt.eval(txt)
    except Exception, e:
        print JSRuntimeError(ctxt, e).full()
    
    if IS_REPL:
        from repl import REPL
        REPL(translator.compiler, linker, translator_arguments, g, ctxt)()


if __name__ == '__main__':
    main()

