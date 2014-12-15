# PROBABILISTIC ROBOT ACTION CORES - LEARNING
#
# (C) 2012 by Daniel Nyga (nyga@cs.tum.edu)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from prac.learning import PRACLearning
from optparse import OptionParser
from prac.core import PRAC
import logging

usage = 'Usage: praclearn [--core <actioncore1>[,<actioncore2>[,...]]] [--module <module1>[,<module2>[,...]]]'

def parse_list(option, opt, value, parser):
    setattr(parser.values, option.dest, value.split(','))

parser = OptionParser(usage=usage)
parser.add_option('--mt', action='callback', type='string', callback=parse_list, dest='microtheories')
parser.add_option('--module', action='callback', type='string', callback=parse_list, dest='modules')
parser.add_option('--dbs', action='callback', type='string', callback=parse_list, dest='training_dbs')
parser.add_option('--mln', type='string', nargs=2, dest='mln', default=None)
parser.add_option('--onthefly', dest='onthefly', default=False, action='store_true', help="Generates MLN on the fly. No learning")    


if __name__ == '__main__':
    (options, args) = parser.parse_args()
    prac = PRAC()
    logging.getLogger().setLevel(logging.INFO)
    praclearn = PRACLearning(prac)
    praclearn.microtheories = parser.values.microtheories
    praclearn.modules = parser.values.modules
    
    if praclearn.microtheories is None:
        praclearn.microtheories = prac.microtheories
    if praclearn.modules is None:
        praclearn.modules = ['prop_extraction']
    if parser.values.training_dbs is not None:
        dbnames = parser.values.training_dbs
        praclearn.training_dbs = dbnames
    if options.mln is not None:
        praclearn.otherParams['mln'] = options.mln[0]
        praclearn.otherParams['logic'] = options.mln[1]
        praclearn.otherParams['onthefly'] = options.onthefly

    for m in praclearn.modules:
        module = prac.getModuleByName(m)
        module.train(praclearn)
    
