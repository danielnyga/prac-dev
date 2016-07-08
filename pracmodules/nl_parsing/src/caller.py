import sys
import os
import jpype
from prac.core.base import prac_module_path
from pracmln import MLN
from pracmodules import StanfordParser
from prac import java

PRAC_HOME = os.environ['PRAC_HOME']
java.classpath.append(os.path.join(PRAC_HOME, '3rdparty',
                                   'stanford-parser-2012-02-03',
                                   'stanford-parser.jar'))
grammarPath = os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2012-02-03',
                           'grammar', 'englishPCFG.ser.gz')


if __name__ == '__main__':


    mln = MLN(mlnfile=os.path.join(prac_module_path, 'nl_parsing', 'mln',
                                   'predicates.mln'),
              grammar='PRACGrammar', logic='FuzzyLogic')

    instr = sys.argv[1:]

    if not java.isJvmRunning():
        java.startJvm()

    if not jpype.isThreadAttachedToJVM():
        jpype.attachThreadToJVM()

    stanford_parser = StanfordParser(grammarPath)

    for s in instr:
        deps = stanford_parser.get_dependencies(s, True)
        deps = map(str, deps)
        words = set()
        for d in deps:
            print d
            f = mln.logic.parse_formula(str(d))
            words.update(f.args)
        postags = stanford_parser.get_pos()
        pos = []
        for pos in postags.values():
            if not pos[0] in words:
                continue
            postagatom = 'has_pos({},{})'.format(pos[0], pos[1])
            pos.append(postagatom)
            print postagatom
            postags[pos[0]] = pos[1]
        print '---'
