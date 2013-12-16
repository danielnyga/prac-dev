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

import sys
from optparse import OptionParser
from prac.core import PRAC
import logging
from actioncore.inference import PRACInference
# import linguistics
# from linguistics.verbalizer import *
# from linguistics.integrator import *

if __name__ == '__main__':
    
    parser = OptionParser()
    parser.add_option('-a', '--add', action='store_true', dest='addToModels')
    parser.add_option('-m', '--map', dest='semanticMap')
    (options, args) = parser.parse_args()
    
#     if not (len(sys.argv) == 4 or len(sys.argv) == 3):
#         print 'Usage: pracinfer <action core name> <sentence> <reply (optional)>\nExample: $pracinfer Flipping "Flip the pancake." "with the spatula"'
#         exit(1)
#     else:        
    log = logging.getLogger()
    log.setLevel(logging.INFO)
    prac = PRAC()
    infer = PRACInference(prac, sys.argv[1:])
    for arg in sys.argv[1:]:
        log.info(arg)
        prac.infer('nl_parsing', infer)
        prac.infer('wn_senses', infer)
        prac.infer('ac_recognition', infer)
        prac.infer('senses_and_roles', infer)
        
#         java.startJvm()
#         sentence = sys.argv[2]
#         intReply = False
#         if len(sys.argv) == 4:
#             intReply = True
#             reply = sys.argv[3]
# 
#         print 'Running PRAC inference on sentence "%s"' % sentence
#         pracinit = PRACInit(sys.argv[1])
#         result = PRACResult()
# #         verbalizer = PRACVerbalizer()
# #         integrator = PRACIntegrator()
#         pracinit(sentence) >> actionroles >> result# >> verbalizer >> integrator
#         
#         java.shutdownJvm()
#         
#         exit(0)
            
