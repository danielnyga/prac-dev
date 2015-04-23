import os
import sys
sys.path.append(os.path.join('praccore'))
from utils import colorize
from subprocess import Popen
home = os.path.abspath(".")
appfolder = "apps"

BOLD = (None, None, True)

if not os.path.exists(appfolder):
    os.mkdir(appfolder)
for app in ("pracinfer", "praclearn", 'pracparse', 'senses', 'pracserver', 'pracobjrec','pracxfold'):
    filepath = os.path.join(appfolder, app)
    f = file(filepath, "w+")
    f.write("#!/bin/sh\n")
    f.write('cd %s\n' % home)
    f.write("python %s \"$@\"\n" % os.path.join(home, "praccore", "%s.py" % app))
    f.close()
    print '\t Wrote app %s' % colorize(app, BOLD, True)
    os.system("chmod a+x %s" % filepath)

# filepath = os.path.join(appfolder, "knowrob")
# f = file(filepath, "w+")
# f.write("#!/bin/sh\n")
# f.write('if [ ! -d ' + os.path.join('/', 'opt', 'ros') + ''' ]; then
#     echo "ROS is not installed. Cannot start KNOWROB. Exiting."    
# else
#    exec rosrun rosprolog run_with_prolog_env mod_vis $(rospack find json_prolog)/bin/json_prolog &
# fi
# ''')
f.close()
os.system("chmod a+x %s" % filepath)
# print os.getcwd()
# print '\t Wrote app %s%s%s' % colorize('knowrob', BOLD)

print '\t Building %s...' % colorize('pyjamas', BOLD, True)
cwd = os.path.join('3rdparty', 'pyjs')
Popen('python bootstrap.py', shell=True, cwd=cwd).wait()
print '\t Building %s...' % colorize('pracviz', BOLD, True)
cmd = 'PYJSPATH=$PYJSPATH:../../praccore/src %s --output=%s pracviz.py' % (os.path.join('..', '..', '3rdparty', 'pyjs', 'bin', 'pyjsbuild'), 
                                                                           os.path.join('output'))
print cmd
#Popen(cmd, cwd=os.path.join('pracviz', 'src'), shell=True).wait()



f = file("env.sh", "w+")
f.write("#!/bin/sh\n")
f.write("export PYTHONPATH=%s\n" % os.path.pathsep.join(["$PYTHONPATH", 
                                                        os.path.join(home, "praccore"), 
                                                        os.path.join(home, "pracGUI"), 
                                                        os.path.join(home, "semcore_annotation_tool", "src"), 
                                                        os.path.join('3rdparty', 'webpy_jsonrpc'),
                                                        os.path.join('3rdparty', 'dill-0.2b1'),
                                                        os.path.join('3rdparty', 'urwid-1.2.0')]))
f.write("export PATH=%s\n" % os.path.pathsep.join(["$PATH", os.path.join(home, appfolder)]))
f.write("export PRAC_HOME=%s\n" % home)
# f.write("export SWI_HOME_DIR=%s\n" % os.path.join('/', 'usr', 'lib', 'swi-prolog'))
f.close()

print '''Apps have been successfully built. Now type "source env.sh" to append them to your PATH.
Note: You may want to add this command to your shell initialization script (e.g. '~/.bashrc')'''
