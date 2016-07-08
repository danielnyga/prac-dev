"""
Created on Mar 22, 2016

@author: picklum
"""
import re
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
from prac.pracutils.RolequeryHandler import RolequeryHandler
from pracmln.mln import NoConstraintsError
from pracmln.mln.util import colorize
import time


prac = None


def test_prac_pipeline_simple(sentences):
    print
    print colorize('+========================+', (None, 'yellow', True), True)
    print colorize('|     INFERENCE TEST     |', (None, 'yellow', True), True)
    print colorize('+========================+', (None, 'yellow', True), True)
    print colorize(', '.join(sentences), (None, 'white', True), True)
    print

    prac.wordnet = WordNet(concepts=None)
    inference = PRACInference(prac, sentences)

    while inference.next_module() is not None:
        modulename = inference.next_module()
        module = prac.getModuleByName(modulename)
        prac.run(inference, module)

    print_results(inference)


def print_results(inference):
    print
    print colorize('+========================+', (None, 'green', True), True)
    print colorize('| PRAC INFERENCE RESULTS |', (None, 'green', True), True)
    print colorize('+========================+', (None, 'green', True), True)

    step = inference.inference_steps[-1]
    wordnet_module = prac.getModuleByName('wn_senses')
    for db in step.output_dbs:
        for a in sorted(db.evidence.keys()):
            v = db.evidence[a]
            if v > 0.001 and (a.startswith('action_core') or a.startswith(
                    'has_sense') or a.startswith('achieved_by')):
                if a.startswith('has_sense'):

                    group = re.split(',', re.split('has_sense\w*\(|\)', a)[1])
                    word = group[0]
                    sense = group[1]
                    if sense != 'null':
                        print
                        print colorize('  WORD:',
                                       (None, 'white', True),
                                       True), word,
                        print colorize('  SENSE:',
                                       (None, 'white', True),
                                       True), sense
                        wordnet_module.printWordSenses(
                            wordnet_module.get_possible_meanings_of_word(db,
                                                                         word),
                            sense)
                        print
                else:
                    print '%.3f    %s' % (v, a)
        RolequeryHandler(prac).queryRolesBasedOnActioncore(db).write(color=True)

    if hasattr(inference.inference_steps[-1], 'executable_plans'):
        print
        print colorize('+==========================+',
                       (None, 'green', True), True)
        print colorize('| PARAMETERIZED ROBOT PLAN |',
                       (None, 'green', True), True)
        print colorize('+==========================+',
                       (None, 'green', True), True)
        print
        for plan in step.executable_plans:
            print plan


def runall():
    start = time.time()
    test_prac_pipeline_simple(['start the centrifuge.'])
    test_prac_pipeline_simple(['Neutralize the methacrylic_acid with 100 '
                               'milliliters of cyanuramide.'])
    print
    print 'all test finished after', time.time() - start, 'secs'
    print

if __name__ == '__main__':
    prac = PRAC()
    runall()
