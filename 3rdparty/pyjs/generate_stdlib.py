#!/usr/bin/env python
"""
Generate stdlib/ from pyjs/lib, /usr/lib/python and pypy/lib
"""

import os
import sys
import shutil
from os.path import join, dirname, basename, abspath, exists, isdir
from optparse import OptionParser

root_path = dirname(abspath(__file__))
dest_path = join(root_path, 'stdlib')

imports = [
    ('pyjs', join(root_path, 'pyjs', 'src', 'pyjs', 'lib'), ['output', 'test']),
    ('pyjs', join(root_path, 'pyjs', 'src', 'pyjs', 'lib_trans'), ['output', 'test']),
    ]

pypy_excludes = ['site-packages', 'pypy_test']
cpython_excludes = ['site-packages', 'lib-dynload', 'test']

mod_src = {}
def copy_tests(dest, src):
    dest = join(dest, 'test')
    src = join(src, 'test')
    if not exists(src):
        return
    if not isdir(src):
        return
    for p in os.listdir(src):
        if (not isdir(join(src, p)) 
            and p.endswith('.py')
            and not exists(join(dest, p))):
            shutil.copy2(join(src, p), join(dest, p))
    
def copy_libs(dest, src, src_name, excludes):
    for p in os.listdir(src):
        if isdir(join(src, p)):
            mod_name = p
        elif p.endswith('.py'):
            mod_name = p.split('.')[0]
        else:
            continue
        
        if (not exists(join(dest, mod_name)) and 
            not exists(join(dest, mod_name + '.py')) and
            not mod_name in excludes):
            if isdir(join(src, p)):
                shutil.copytree(join(src, p), join(dest, p))
            else:
                shutil.copy2(join(src, p), join(dest, p))
            mod_src[mod_name] = src_name
            

def main():
    parser = OptionParser()
    parser.add_option(
        "--pypy",
        dest="pypy",
        help="Path to PyPy libraries")
    parser.add_option(
        "--cpython",
        dest="cpython",
        help="Path to CPython libraries")
    
    options, args = parser.parse_args()
    if not options.cpython:        
        cpython_path = dirname(os.__file__)
    else:
        cpython_path = options.cpython
    
    if options.pypy:
        imports.append(('pypy', options.pypy, pypy_excludes))
    
    if cpython_path:
        imports.append(('cpython', cpython_path, cpython_excludes))
        
    if exists(dest_path):
        shutil.rmtree(dest_path)
    os.mkdir(dest_path)
    os.mkdir(join(dest_path, 'test'))
    
    print ("Exporting data from:")
    for name, path, exc in imports:
        print ("{0}: {1}".format(name, path))  
        copy_libs(dest_path, path, name, exc)
        copy_tests(dest_path, path)

    f_mod_src = open(join(dest_path, 'modules_sources'), 'w')
    for mod, src in mod_src.iteritems():
        f_mod_src.write("{}:{}\n".format(mod, src))
    f_mod_src.close()
        
    
if __name__ == '__main__':
    main()