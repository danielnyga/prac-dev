from spatialreasoning import *
from  spatialreasoning.dbgenerator import *
import sys
import java
from actioncore import PRAC
from actioncore.inference import *
from optparse import OptionParser
if __name__ == '__main__':
    
    parser = OptionParser()
    parser.add_option('-a', '--add', action='store_true', dest='addToModels')
    parser.add_option('-m', '--map', dest='semanticMap')
    (options, args) = parser.parse_args()
    '''
    if not len(sys.argv) == 3:
        print 'Usage: pracinfer <action core name> <sentence>\nExample: $pracinfer Flipping "Flip the pancake."'
        exit(1)
    else: 
    '''
    #java.startJvm()
    
    #type=sys.argv[2]
    #sentence = sys.argv[1]
    
    #type='Flipping'
    #sentence='Fill a bottle with oil from the tap'
    #sentence='Fill a mug with chocolate from the kettle'
    #sentence='Fill a glass with milk from the pot'
    #sentence='Fill a cup with water from the tap'
    
    type='Filling'
    sentence='Push the spatula under the pancake'
    
    # here code for database generation
    '''
    pracinit = PRACInit(type)
    result = PRACResult()
    spatial_db_generator=DatabaseGenerator(type)
    pracinit(sentence) >> actionroles >> result >> spatial_db_generator
    java.shutdownJvm()
    '''
    
    # here code for learning
    '''
    prac = PRAC()
    action_core = prac.action_cores[type]
    learner = SPATLearner(action_core)
    learner.run()
    prac.write_spatial(action_core)
    '''
    
    #here code for inference
    db_infer=SPATInference(type)
    db_infer.run('tapwatcup')
    
    exit(0)
        