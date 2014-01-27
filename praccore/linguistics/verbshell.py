
import cmd
import os
from linguistics import HRIDialog

class VerbShell(cmd.Cmd): 


    def __init__(self):
        self.dialog = HRIDialog() 
        last_output = ''

    def do_shell(self, line):
        "Run a shell command"
        print "running shell command:", line
        output = os.popen(line).read()
        print output
        self.last_output = output
    
    def do_echo(self, line):
        "Print the input, replacing '$out' with the output of the last shell command"
        # Obviously not robust
        print line.replace('$out', self.last_output)

    def do_trial(self, line):
        dialog.add['PR2'] = line
        print dialog.getLast['PR2']
    
    def do_exit(self, line):
        "exit the PRAC Human-Robot Interaction Shell"
        return True
