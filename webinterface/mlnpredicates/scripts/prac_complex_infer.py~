#!/usr/bin/env python
# PROBABILISTIC ROBOT ACTION CORES - ROS INTERFACE 
#
# (C) 2013 by Daniel Nyga (nyga@cs.tum.edu)
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

import roslib; roslib.load_manifest('rosprac')

from rosprac.srv import *
from rosprac.msg import *
from rosmln.msg import *
import rospy
import os
import sys

PRAC_HOME = os.environ.get('PRAC_HOME')
PROBCOG_HOME = os.environ.get('PROBCOG_HOME')
sys.path.append(PRAC_HOME)
sys.path.append(os.path.join(PROBCOG_HOME, 'python'))

from rospracutils import DatabasesFromROSMsg
from prac.core import PRAC
from prac.inference import PRACInference

import jpype
import java

java.classpath.append(os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2012-02-03', 'stanford-parser.jar'))
grammarPath = os.path.join(PRAC_HOME, '3rdparty', 'stanford-parser-2012-02-03', 'grammar', 'englishPCFG.ser.gz')

prac = PRAC()
pub_pracinfer = None

def pracinfer_handler(param):
    if len(param.instructions) > 0:
        print 'Running PRAC Inference on instructions "%s"' % (param.instructions)
    if len(param.input_dbs) > 0:
        for db in param.input_dbs:
            print db
        
    if not java.isJvmRunning():
        java.startJvm()
    if not jpype.isThreadAttachedToJVM():
        jpype.attachThreadToJVM()

    # setup the inference object
    infer = PRACInference(prac, param.instructions)
    module  = prac.getModuleByName(param.pracmodule)
    module.insertdbs(infer, *list(DatabasesFromROSMsg(prac.mln, *param.input_dbs)))

    # construct the input dbs and publish them
    for dbs in infer.inference_steps[-1].output_dbs:
        db_msg = MLNDatabase()
        atom_prob_pairs = []
        for atom, value in dbs.evidence.iteritems():
            pair = AtomProbPair()
            pair.atom = atom
            pair.prob = value
            atom_prob_pairs.append(pair)
        db_msg.evidence = atom_prob_pairs
        pub_pracinfer.publish(db_msg)
    
    # run the module on the inference
    prac.run(infer, module, **eval('dict(%s)' % param.params))
    
    output_dbs = []
    for dbs in infer.inference_steps[-1].output_dbs:
        db_msg = MLNDatabase()
        atom_prob_pairs = []
        for atom, value in dbs.evidence.iteritems():
            pair = AtomProbPair()
            pair.atom = atom
            pair.prob = value
            atom_prob_pairs.append(pair)
        db_msg.evidence = atom_prob_pairs
        pub_pracinfer.publish(db_msg)
        output_dbs.append(db_msg)
        
#     for kb in module.kbs:
#         mln = kb.query_mln
#         for db in kb.dbs:
#             mrf = mln.groundMRF(db)
#             cliques = []
#             evidence = list(db.evidence.keys())
#             for f in mrf.gndFormulas:
#                 clique = MRFClique()
#                 atoms = sorted(f.getGroundAtoms())
#                 atoms = map(str, atoms)
#                 clique.variables = atoms
#                 cliques.append(clique)
#             mlnInference = MLNInference()
#             mlnInference.evidence = evidence
#             mlnInference.cliques = cliques
#             pub_pracmln.publish(mlnInference)
        
    return (output_dbs,)

def prac_server():
    rospy.init_node('pracinfer')
    s = rospy.Service('PRACInfer', PRACInfer, pracinfer_handler)
    print "PRAC service is running. Waiting for queries..."
    rospy.spin()

if __name__ == "__main__":
    # create the publisher
    pub_pracinfer = rospy.Publisher('rosprac/pracinfer', MLNDatabase)
    pub_pracmln = rospy.Publisher('rosprac/pracmln', MLNInference)
    prac_server()
