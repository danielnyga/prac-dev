import sys
import PyV8

class FileWrapper(object):
    def __init__(self, fname, mode):
        self.f = open(fname, mode)

    def seek(self, seekto=None):
        if seekto is None:
            return self.f.seek()
        return self.f.seek(seekto)

    def close(self):
        return self.f.close()

    def write(self, bytes):
        return self.f.write(bytes)

    def read(self, bytes=None):
        if bytes is None:
            return self.f.read()
        return self.f.read(bytes)

# Create a python class to be used in the context
class Global(PyV8.JSClass):
    def __init__(self, argv=None, path=None):
        PyV8.JSClass.__init__(self)
        
        if argv is None:
            self.argv = sys.argv
        else:
            self.argv = argv
            
        if path is None:
            self.path = sys.path
        else:
            self.path = path
            
    # Funcs to initialize sys
    def pyv8_sys_get_stdin(self):
        return sys.stdin
    
    def pyv8_sys_get_stdout(self):
        return sys.stdout
    
    def pyv8_sys_get_stderr(self):
        return sys.stderr
    
    def pyv8_sys_get_argv(self):
        return self.argv[:]
    
    def pyv8_sys_get_path(self):
        return self.path[:]
    
    def pyv8_open(self, fname, mode):
        return FileWrapper(fname, mode)
   

        

    def pyv8_import_module(self, parent_name, module_name):
        #print "pyv8_import_module", parent_name, module_name
        exec "import " + module_name
        return locals()[module_name]

    def pyv8_load(self, modules):
        for i in range(len(modules)):
            fname = modules[i]
            try:
                fp = open(fname, 'rb')
                txt = fp.read()# historically js files were ascii or latin1 only, we only generate ascii to be safer
                fp.close()
                """ # Use this if encoding problems in generated javascript files
                for index, line in enumerate(txt.split("\n")):
                    assert isinstance(txt, str)
                    try:
                        line.decode("ascii")
                    except UnicodeError:
                        import sys
                        print >> sys.stderr, "BUGGY LINE", index + 1, "-", repr(line)
                """
                x = self.__context__.eval(txt, fname)
            except Exception, e:
                import traceback
                import sys
                traceback.print_exc(file=sys.stderr)
                raise ImportError("Failed to load %s: '%s'" % (fname, e))
