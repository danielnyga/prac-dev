# Initialize system values

def sys_init():
    
    def pyv8_list(l):
        l2 = list()
        for i in range(l.__len__()):
            l2[0:0] = [l.pop()]        
        return l2
        
    global stdin
    stdin = pyv8_sys_get_stdin()
    
    global stdout
    stdout = pyv8_sys_get_stdout()

    global stderr
    stderr = pyv8_sys_get_stderr()

    global argv
    argv = pyv8_list(pyv8_sys_get_argv())

    global path
    path = pyv8_list(pyv8_sys_get_path())

