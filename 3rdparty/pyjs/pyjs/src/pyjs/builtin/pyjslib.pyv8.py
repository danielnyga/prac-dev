def __dynamic_load__(importName):
    setCompilerOptions("noDebug")
    module = JS("""$pyjs.loaded_modules[@{{importName}}]""")
    if JS("""typeof @{{module}} == 'undefined'"""):
        try:
            dynamic.ajax_import("lib/" + importName + ".__" + platform + "__.js")
            module = JS("""$pyjs.loaded_modules[@{{importName}}]""")
        except:
            pass
    if JS("""typeof @{{module}} == 'undefined'"""):
        try:
            dynamic.ajax_import("lib/" + importName + ".js")
            module = JS("""$pyjs.loaded_modules[@{{importName}}]""")
        except:
            pass
    #if JS("""typeof module == 'undefined'"""):
    #    module = pyv8_import_module(None, importName)
    return module

def debugReport(msg):
    print msg

def open(fname, mode='r'):
    return JS("pyv8_open(@{{fname}}, @{{mode}});")