'''
Created on Jul 11, 2016

@author: nyga
'''
from optparse import OptionParser
from pracmln.mln.util import headline
from prac.db.operations import store_howto, analyze_howto
from prac.core.base import PRAC
import os



if __name__ == '__main__':
    #===========================================================================
    # Parse command line arguments
    #===========================================================================
    parser = OptionParser()
    parser.add_option('-H', '--howto', dest='howto', help='Title of the howto, e.g. "Make pancakes"')
    parser.add_option('-s', '--steps', dest='steps', default=False, action='store_true', 
                      help='A list of instruction steps in natural language. If set, his option must be the last in the list of options followed by the list of instructions.')
    parser.add_option('-b', '--batch', dest='batch', default=False, action='store_false',
                      help='Import a list of howtos in batch processing whose filenames are given the respective howto title, e.g. "Make a pancake." The file content must then be given by the single instruction steps, one by line.')
    parser.add_option('-r', '--recursive', dest='recursive', default=False, 
                      help='Apply the import of instructions recursively to subdirectories.')
    parser.add_option("-v", "--verbose", dest="verbose", default=1, type='int', action="store",
                      help="Set verbosity level {0..3}. Default is 1.")
    parser.add_option('-q', '--quiet', dest='quiet', action='store_true', default=False,
                      help='Do not print any status messages.')
    
    (options, args) = parser.parse_args()
    if options.quiet: options.verbose = 0
    
    #===========================================================================
    # If the 'steps' flag is set, take all arguments as the list of instructions 
    #===========================================================================
    howtos = []
    if options.steps:
        howtos = [{options.howto: args}]
    elif options.batch:
        for path in args:
            if options.recursive:
                for loc, dirs, files in os.walk(path):
                    for filename in files:
                        with open(os.path.join(loc, filename)) as f:
                            howtos.append({' '.join(filename.split('-')): f.read().splitlines()})
            else:
                for filename in os.listdir(path):
                    if os.path.isdir(filename): continue
                    with open(os.path.join(path, filename)) as f:
                        howtos.append({' '.join(filename.split('-')): f.read().splitlines()})
    else:
        for filename in args:
            with open(filename) as f:
                howtos.append({' '.join(filename.split('-')): f.read().splitlines()})
    prac = PRAC()
    prac.verbose = options.verbose
    for howto, steps in [(howto, steps) for d in howtos for howto, steps in d.items()]:
        result = analyze_howto(prac, howto, steps, verbose=options.quiet > 0)
        store_howto(prac, result)
    
