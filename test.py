"""
Created on Mar 22, 2016

@author: picklum
"""
import re
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
from prac.pracutils.utils import prac_heading
from pracmln.mln.util import colorize
import time


prac = None


def test_prac_pipeline_simple(sentences):
    print prac_heading('Inference Test', color='yellow')
    print colorize(', '.join(sentences), (None, 'white', True), True)
    print

    prac.wordnet = WordNet(concepts=None)
    inference = PRACInference(prac, sentences)

    while inference.next_module() is not None:
        modulename = inference.next_module()
        module = prac.module(modulename)
        prac.run(inference, module)

    print_results(inference)


def print_results(inference):
    print prac_heading('PRAC Inference Results')

    step = inference.inference_steps[-1]
    wordnet_module = prac.module('wn_senses')
    for db in step.output_dbs:
        for a in sorted(db.evidence.keys()):
            v = db.evidence[a]
            if v > 0.001 and (a.startswith('action_core') or a.startswith('has_sense') or a.startswith('achieved_by')):
                if a.startswith('has_sense'):

                    group = re.split(',', re.split('has_sense\w*\(|\)', a)[1])
                    word = group[0]
                    sense = group[1]
                    if sense != 'null':
                        print
                        print colorize('  WORD:', (None, 'white', True), True), word,
                        print colorize('  SENSE:', (None, 'white', True), True), sense
                        wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(db, word), sense)
                        print
                else:
                    print '{:.3f}    {}'.format(v, a)

        for ac in db.actioncores():
            actioncore = ac.values().pop()
            for d in db.roles(actioncore):
                print '{}({},{})'.format(
                    colorize(d.keys().pop(), (None, 'white', True), True),
                    d.values().pop(), actioncore)

    if hasattr(inference.inference_steps[-1], 'executable_plans'):
        print prac_heading('Parameterized Robot Plan')
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
