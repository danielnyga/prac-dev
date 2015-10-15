#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import stat
import sys
import platform
import shutil
import imp

sys.path.append(os.path.join(os.getcwd(), 'prac'))

from pracmln.mln.util import colorize

packages = [('web', 'webpy'),('bs4','bs4')]

def check_package(pkg):
    try:
        sys.stdout.write('checking dependency %s...' % pkg[0])
        imp.find_module(pkg[0])
        sys.stdout.write(colorize('OK', (None, 'green', True), True))
        print
    except ImportError:
        print
        print colorize('%s was not found. Please install by "sudo apt-get install python-%s"' % (pkg[0], pkg[1]), (None, 'yellow', True), True)

# check the package dependecies
for pkg in packages:
    check_package(pkg)

python_apps = [
    {"name": "pracquery", "script": "$PRAC_HOME/prac/pracquery.py"},
    {"name": "praclearn", "script": "$PRAC_HOME/prac/praclearn.py"},
    {"name": "senses", "script": "$PRAC_HOME/prac/senses.py"},
    {"name": "pracobjrec", "script": "$PRAC_HOME/prac/pracobjrec.py"},
    {"name": "pracxfold", "script": "$PRAC_HOME/prac/pracxfold.py"},
]

def adapt(name, arch):
    return name.replace("<ARCH>", arch).replace("$PRAC_HOME", os.path.abspath(".")).replace("/", os.path.sep)


def build_pracweb():
    # build qooxdoo
    generate = adapt("$PRAC_HOME/pracweb/gui/generate.py", arch)
    os.system(generate + ' source-all')
    os.system(generate + ' build')

    python_apps.append({"name": "pracweb", "script": "$PRAC_HOME/pracweb/run.py"})


if __name__ == '__main__':

    archs = ["win32", "linux_amd64", "linux_i386", "macosx", "macosx64"]

    print "PRAC Apps Generator\n\n"
    print "  usage: make_apps [--arch=%s] [--cppbindings]\n" % "|".join(archs)
    print
    print

    args = sys.argv[1:]

    # determine architecture
    arch = None
    bits = 64 if "64" in platform.architecture()[0] else 32
    if len(args) > 0 and args[0].startswith("--arch="):
        arch = args[0][len("--arch="):].strip()
        args = args[1:]
    elif platform.mac_ver()[0] != "":
        arch = "macosx" if bits == 32 else "macosx64"
    elif platform.win32_ver()[0] != "":
        arch = "win32"
    elif platform.dist()[0] != "":
        arch = "linux_i386" if bits == 32 else "linux_amd64"
    if arch is None:
        print "Could not automatically determine your system's architecture. Please supply the --arch argument"
        sys.exit(1)
    if arch not in archs:
        print "Unknown architecture '%s'" % arch
        sys.exit(1)

    buildlib = False
    if len(args) > 0 and args[0] == "--cppbindings":
        buildlib = True;
        args = args[1:]

    print 'Removing old app folder...'
    shutil.rmtree('apps', ignore_errors=True)

    if not os.path.exists("apps"):
        os.mkdir("apps")

    print "\nCreating application files for %s..." % arch
    isWindows = "win" in arch
    isMacOSX = "macosx" in arch
    preamble = "@echo off\r\n" if isWindows else "#!/bin/sh\n"
    allargs = '%*' if isWindows else '"$@"'
    pathsep = os.path.pathsep

    for app in python_apps:
        filename = os.path.join("apps", "%s%s" % (app["name"], {True:".bat", False:""}[isWindows]))
        print "  %s" % filename
        f = file(filename, "w")
        f.write(preamble)
        f.write("python \"%s\" %s\n" % (adapt(app["script"], arch), allargs))
        f.close()
        if not isWindows: os.chmod(filename, stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)


    if '--pracweb' in args:
        build_pracweb()

    print

    # write shell script for environment setup
    appsDir = adapt("$PRAC_HOME/apps", arch)
    praccoreDir = adapt("$PRAC_HOME/praccore", arch)

    if not "win" in arch:
        f = file("env.sh", "w")
        f.write('#!/bin/bash\n')
        f.write("export PATH=$PATH:%s\n" % appsDir)
        f.write("export PYTHONPATH=$PYTHONPATH:%s\n" % praccoreDir)
        f.write("export PRAC_HOME=%s\n" % adapt("$PRAC_HOME", arch))
        f.write("export PYTHONPATH=$PRAC_HOME:$PYTHONPATH\n")
        print 'Now, to set up your environment type:'
        print '    source env.sh'
        print
        print 'To permantly configure your environment, add this line to your shell\'s initialization script (e.g. ~/.bashrc):'
        print '    source %s' % adapt("$PRAC_HOME/env.sh", arch)
        print
    else:
        f = file("env.bat", "w")
        f.write("SET PATH=%%PATH%%;%s\r\n" % appsDir)
        f.write("SET PRAC_HOME=%s\n" % adapt("$PRAC_HOME", arch))
        f.close()
        print 'To temporarily set up your environment for the current session, type:'
        print '    env.bat'
        print
        print 'To permanently configure your environment, use Windows Control Panel to set the following environment variables:'
        print '  To the PATH variable add the directory "%s"' % appsDir
        print 'Should any of these variables not exist, simply create them.'
