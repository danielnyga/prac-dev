import os
import sys
sys.path.append(os.path.join('edu.tum.cs.prac', 'src'))
from utils import bash

home = os.path.abspath(".")
appfolder = "apps"

if not os.path.exists(appfolder):
    os.mkdir(appfolder)
for app in ("pracinfer", "praclearn", 'pracparse', 'senses'):
    filepath = os.path.join(appfolder, app)
    f = file(filepath, "w+")
    f.write("#!/bin/sh\n")
    f.write('cd %s\n' % home)
    f.write("python %s \"$@\"\n" % os.path.join(home, "edu.tum.cs.prac", "src", "%s.py" % app))
    f.close()
    print '\t Wrote app %s%s%s' % (bash.BOLD, app, bash.END)
    os.system("chmod a+x %s" % filepath)
    
filepath = os.path.join(appfolder, "knowrob")
f = file(filepath, "w+")
f.write("#!/bin/sh\n")
f.write('if [ ! -d ' + os.path.join('/', 'opt', 'ros') + ''' ]; then
    echo "ROS is not installed. Cannot start KNOWROB. Exiting."    
else
   exec rosrun rosprolog run_with_prolog_env mod_vis $(rospack find json_prolog)/bin/json_prolog &
fi
''')
f.close()
os.system("chmod a+x %s" % filepath)

print '\t Wrote app %s%s%s' % (bash.BOLD, 'knowrob', bash.END)

f = file("env.sh", "w+")
f.write("#!/bin/sh\n")
f.write("export PYTHONPATH=%s\n" % os.path.pathsep.join(["$PYTHONPATH", os.path.join(home, "edu.tum.cs.prac", "src"), os.path.join(home, "semcore_annotation_tool", "src")]))
f.write("export PATH=%s\n" % os.path.pathsep.join(["$PATH", os.path.join(home, appfolder)]))
f.write("export PRAC_HOME=%s\n" % home)
f.write("export SWI_HOME_DIR=%s\n" % os.path.join('/', 'usr', 'lib', 'swi-prolog'))
f.close()

print '''Apps have been successfully built. Now type "source env.sh" to append them to your PATH.
Note: You may want to add this command to your shell initialization script (e.g. '~/.bashrc')'''
