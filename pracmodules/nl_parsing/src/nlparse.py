import os
import jpype
from pracmln import MLN
from pracmodules import StanfordParser
from prac import java
import prac
import json
from optparse import OptionParser

#===============================================================================
# set up JVM classpath
#===============================================================================
java.classpath.append(os.path.join(prac.locations.home, 
                                   '3rdparty',
                                   'stanford-parser-2015-12-09',
                                   'stanford-parser.jar'))
java.classpath.append(os.path.join(prac.locations.home, 
                                   '3rdparty',
                                   'stanford-parser-2015-12-09',
                                   'slf4j-api.jar'))

#===============================================================================
# path to the grammar
#===============================================================================

grammar_path = os.path.join(prac.locations.home, 
                           '3rdparty', 
                           'stanford-parser-2015-12-09',
                           'grammar', 
                           'englishPCFG.ser.gz')

def main(args, options):
    #===========================================================================
    # Load the NL parsing MLN
    #===========================================================================
    mln = MLN(mlnfile=os.path.join(prac.locations.modules, 'nl_parsing', 'mln', 'predicates.mln'),
              grammar='PRACGrammar', logic='FuzzyLogic')

    #===========================================================================
    # Load the Java VM
    #===========================================================================
    if not java.isJvmRunning():
        java.startJvm()
    if not jpype.isThreadAttachedToJVM():
        jpype.attachThreadToJVM()
    
    #===========================================================================
    # # suppress the stderr outputs from the parser
    #===========================================================================
    jpype.java.lang.System.setErr(jpype.java.io.PrintStream(os.devnull))
    
    #===========================================================================
    # Initialize the parser
    #===========================================================================
    stanford_parser = StanfordParser(grammar_path)
    dbs = []
    sentences = args
    for s in sentences:
        db = ''
        deps = stanford_parser.get_dependencies(json.loads(s), True)
        deps = map(str, deps)
        words = set()
        for d in deps:
            db += '%s\n' % d
            f = mln.logic.parse_formula(str(d))
            words.update(f.args)
        postags = stanford_parser.get_pos()
        pos = []
        for pos in postags.values():
            if not pos[0] in words:
                continue
            postagatom = 'has_pos({},{})'.format(pos[0], pos[1])
            pos.append(postagatom)
            db += '%s\n' % postagatom
            postags[pos[0]] = pos[1]
        dbs.append(db)
    result = '---\n'.join(dbs)
    if options.outfile is not None:
        with open(options.outfile, 'w+') as f:
            f.write(result)
    else:
        print result,

#===============================================================================
# command line arguments declaration
#===============================================================================
parser = OptionParser(description='Parse natural-language sentences and return the MLN databases (uses the Stanford Parser).')
parser.add_option('-o', '--out-file', dest='outfile', default=None, help='the file to write the results to.')


if __name__ == '__main__':
    options, args = parser.parse_args()
    main(args, options)
    

    