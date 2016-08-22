#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import stat
import sys
import platform
import shutil
import imp
import apt

import pymongo


try:
    from pracmln.mln.util import colorize
except ImportError:
    print
    print "Could not find PRACMLN. Please install PRACMLN first using the " \
          "make_apps.py in your PRACMLN root directory!"
    print
    sys.exit(-1)


env_vars = ['JAVA_HOME']
aptpackages = ['dvipng', 'texlive-full', 'default-jre', 'mongodb-server']

# terminology: (nametobechecked, pip-packagename, optionalpackage?)
packages = [('sphinx', 'sphinx sphinxcontrib-bibtex', False),
            ('jpype', 'jpype1', False),
            ('nltk', 'nltk', False),
            ('graphviz', 'graphviz', False),
            ('bs4', 'beautifulsoup4', False),
            ('lxml', 'lxml', False),
            ('yaml', 'pyyaml', False),
            ('matplotlib', 'matplotlib', False),
            ('apt', 'python-apt', False),
            ('pymongo', 'pymongo', False), # version check in initialize_mongodb
            ('num2words', 'num2words', False),
            ('word2number', 'word2number', False)
            ]

python_apps = [
    {"name": "pracquery", "script": "$PRAC_HOME/prac/pracquery.py"},
    {"name": "praclearn", "script": "$PRAC_HOME/prac/praclearn.py"},
    {"name": "practell", "script": "$PRAC_HOME/prac/practell.py"},
    {"name": "senses", "script": "$PRAC_HOME/prac/senses.py"},
    {"name": "pracobjrec", "script": "$PRAC_HOME/prac/pracobjrec.py"},
    {"name": "pracxfold", "script": "$PRAC_HOME/prac/pracxfold.py"},
    {"name": "pracparse", "script": "$PRAC_HOME/prac/pracparse.py"},
]


def check_package(pkg):
    try:
        sys.stdout.write('checking dependency {}...'.format(pkg[0]))
        imp.find_module(pkg[0])
        sys.stdout.write(colorize('OK', (None, 'green', True), True))
        print
    except ImportError:
        print
        print colorize('{} was not found. Please install by '
                       '"sudo pip install {}" {}'.format(pkg[0], pkg[1],
                                                         '(optional)' if pkg[2] else ''),
                       (None, 'yellow', True), True)


# check the package dependecies
def check_dependencies():
    # python packages
    print colorize('Checking python package dependencies...',
                   (None, 'green', True), True)
    for pkg in packages:
        check_package(pkg)
    print

    # debian packages
    print colorize('Checking debian package dependencies...',
                   (None, 'green', True), True)
    cache = apt.Cache()
    for pkg in aptpackages:
        sys.stdout.write('checking dependency {}...'.format(pkg))
        if not cache[pkg].is_installed:
            print
            print colorize('{} was not found. Please install by '
                           '"sudo apt-get install {}"'.format(pkg, pkg),
                           (None, 'yellow', True), True)
        else:
            sys.stdout.write(colorize('OK', (None, 'green', True), True))
            print
    print

    # environment variables
    print colorize('Checking necessary environment variables...',
                   (None, 'green', True), True)
    for var in env_vars:
        sys.stdout.write('checking environment variable {}...'.format(var))
        if var not in os.environ:
            print colorize('{} was not found in your environment variables.'.format(var), (None, 'yellow', True), True)
        else:
            sys.stdout.write(colorize('OK', (None, 'green', True), True))
            print
    print


def adapt(name, arch):
    return name.replace("<ARCH>", arch)\
               .replace("$PRAC_HOME", os.path.abspath("."))\
               .replace("/", os.path.sep)


def initialize_mongodb():
    print colorize('Initializing Mongo DB...', (None, 'green', True), True)
    try:
        # check if mongo server version and pymongo version are suitable
        assert pymongo.MongoClient().server_info()['versionArray'] > [3,0,0,0] and pymongo.version_tuple > (3,0,0)
        # download files and initialize db
        os.system('wget http://ai.uni-bremen.de/public/prac/howtos.bson && mongorestore --db prac --collection howtos howtos.bson')
        # cleanup
        os.remove('howtos.bson')
    except AssertionError:
        print colorize('Both Mongo server version and pymongo version must be >=3.X.', (None, 'red', True), True)
        print 'Your Mongo Server version: \t{}\nYour Pymongo version: \t\t{}'.format('.'.join(str(x) for x in pymongo.MongoClient().server_info()['versionArray']),
                                                                                     '.'.join(str(x) for x in pymongo.version_tuple))


if __name__ == '__main__':

    archs = ["win32", "linux_amd64", "linux_i386", "macosx", "macosx64"]

    args = sys.argv[1:]

    if '--help' in args:
        print "PRAC Apps Generator\n\n"
        print "  usage: make_apps [--arch={}]\n".format("|".join(archs))
        print
        exit(0)

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
        print "Could not automatically determine your system's architecture." \
              " Please supply the --arch argument"
        sys.exit(1)
    if arch not in archs:
        print "Unknown architecture '{}'".format(arch)
        sys.exit(1)

    check_dependencies()

    initialize_mongodb()

    print 'Removing old app folder...'
    shutil.rmtree('apps', ignore_errors=True)

    if not os.path.exists("apps"):
        os.mkdir("apps")

    print "\nCreating application files for {}...".format(arch)
    isWindows = "win" in arch
    isMacOSX = "macosx" in arch
    preamble = "@echo off\r\n" if isWindows else "#!/bin/sh\n"
    allargs = '%*' if isWindows else '"$@"'
    pathsep = os.path.pathsep

    for app in python_apps:
        filename = os.path.join("apps", "{}{}".format(app["name"], {True: ".bat", False: ""}[isWindows]))
        print "  {}".format(filename)
        f = file(filename, "w")
        f.write(preamble)
        f.write("python \"{}\" {}\n".format(adapt(app["script"], arch), allargs))
        f.close()
        if not isWindows:
            os.chmod(filename,
                     stat.S_IRUSR |
                     stat.S_IWUSR |
                     stat.S_IXUSR |
                     stat.S_IRGRP |
                     stat.S_IXGRP |
                     stat.S_IROTH |
                     stat.S_IXOTH)
    print

    # write shell script for environment setup
    appsDir = adapt("$PRAC_HOME/apps", arch)
    nltkdir = adapt("$PRAC_HOME/3rdparty/nltk_2.0b9", arch)

    if "win" not in arch:
        f = file("env.sh", "w")
        f.write('#!/bin/bash\n')
        f.write("export PATH=$PATH:{}\n".format(appsDir))
        f.write("export PYTHONPATH=$PYTHONPATH:{}\n".format(nltkdir))
        f.write("export PRAC_HOME={}\n".format(adapt("$PRAC_HOME", arch)))
        f.write("export PYTHONPATH=$PRAC_HOME:$PYTHONPATH\n")
        print 'Now, to set up your environment type:'
        print '    source env.sh'
        print
        print 'To permantly configure your environment, add this line to ' \
              'your shell\'s initialization script (e.g. ~/.bashrc):'
        print '    source {}'.format(adapt("$PRAC_HOME/env.sh", arch))
        print
    else:
        pypath = ';'.join([adapt("$PRAC_HOME", arch), nltkdir])
        f = file("env.bat", "w")
        f.write("@ECHO OFF\n")
        f.write('SETX PATH "%%PATH%%;{}"\r\n'.format(appsDir))
        f.write('SETX PRAC_HOME "{}"\r\n'.format(adapt("$PRAC_HOME", arch)))
        f.write('SETX PYTHONPATH "%%PYTHONPATH%%;{}"\r\n'.format(pypath))
        f.close()
        print 'To temporarily set up your environment for the current ' \
              'session, type:'
        print '    env.bat'
        print
        print 'To permanently configure your environment, use Windows ' \
              'Control Panel to set the following environment variables:'
        print '  To the PATH variable add the directory "{}"'.format(appsDir)
        print 'Should any of these variables not exist, simply create them.'
