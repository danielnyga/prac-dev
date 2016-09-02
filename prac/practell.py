'''
Created on Jul 11, 2016

@author: nyga
'''
from optparse import OptionParser
import os
from prac.pracutils.utils import prac_heading
import multiprocessing
from prac.core.base import PRAC
import itertools
from pracmln.utils import multicore
import traceback




if __name__ == '__main__':
    #===========================================================================
    # Parse command line arguments
    #===========================================================================
    parser = OptionParser()
    parser.add_option('-H', '--howto', dest='howto', help='Title of the howto, e.g. "Make pancakes"')
    parser.add_option('-s', '--steps', dest='steps', default=False, action='store_true', 
                      help='A list of instruction steps in natural language. If set, his option must be the last in the list of options followed by the list of instructions.')
    parser.add_option('-b', '--batch', dest='batch', default=False, action='store_true',
                      help='Import a list of howtos in batch processing whose filenames are given the respective howto title, e.g. "Make a pancake." The file content must then be given by the single instruction steps, one by line.')
    parser.add_option('-r', '--recursive', dest='recursive', default=False, 
                      help='Apply the import of instructions recursively to subdirectories.')
    parser.add_option("-v", "--verbose", dest="verbose", default=1, type='int', action="store",
                      help="Set verbosity level {0..3}. Default is 1.")
    parser.add_option('-q', '--quiet', dest='quiet', action='store_true', default=False,
                      help='Do not print any status messages.')
    parser.add_option('-m', '--multicore', dest='multicore', action='store_true', default=False,
                      help='Perform information extraction in multicore modus')
    
    (options, args) = parser.parse_args()
    if options.quiet: options.verbose = 0
    
    if options.verbose:
        print prac_heading('Telling PRAC, how to {}'.format(options.howto))

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
                            howtos.append({' '.join(filename.split('-')): filter(None, (line.strip() for line in f))})
            else:
                for filename in os.listdir(path):
                    if os.path.isdir(filename): continue
                    with open(os.path.join(path, filename)) as f:
                        howtos.append({' '.join(filename.split('-')): filter(None, (line.strip() for line in f))})
    else:
        for filename in args:
            with open(filename) as f:
                howtos.append({' '.join(filename.split('-')): filter(None, (line.strip() for line in f))})

    #===========================================================================
    # start the import 
    #===========================================================================
    def import_howto(args):
        try:
            howto_steps, verbose = args
            prac = PRAC()
            prac.verbose = verbose
            for howto, steps in howto_steps.iteritems():
                prac.tell(howto=howto, steps=steps)
        except KeyboardInterrupt: return
    try:
        cpu_count =  multiprocessing.cpu_count() if options.multicore else 1
        pool = multicore.NonDaemonicPool(cpu_count)
        pool.map(multicore.with_tracing(import_howto), itertools.izip(howtos, itertools.repeat(options.verbose)))
    except KeyboardInterrupt:
        traceback.print_exc()
        pool.terminate()
    else:
        #=======================================================================
        # finished
        #=======================================================================
        if options.verbose: print 'Done. Imported %d howtos' % len(howtos)
    finally:
        pool.close()
        pool.join()
        
    
     