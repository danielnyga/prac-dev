import nltk
from nltk.corpus import wordnet as wn
import roslib; roslib.load_manifest('json_prolog')
import os
import rospy
import sys
import json_prolog
import java
import jpype
from StanfordParser import Parser
import re
from WordSenseFeatureGenerator import WordSenseFeatureGenerator
from grammar import parsePracFormula
from actioncore import PRAC
from inference2 import PRACInference
from optparse import OptionParser


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-a', '--add', action='store_true', dest='addToModels')
    parser.add_option('-m', '--map', dest='semanticMap')
    (options, args) = parser.parse_args()
    
    if not len(sys.argv) == 1:
        print 'Usage: pracinfer <sentence>\nExample: $pracinfer "Push the spatula under the pancake."'
        exit(1)
    else: 
        java.startJvm()
        #sentence = sys.argv[1]
        sentence='Fill a bottle with oil from the tap'
        #sentence='Fill a mug with chocolate from the kettle'
        #sentence='Fill a glass with milk from the pot'
        #sentence='Fill a cup with water from the tap'
        print 'Running PRAC inference on sentence "%s"' % sentence
        prac = PRAC()
        
        infer = PRACInference(prac)
        
        infer.semanticMap = options.semanticMap 
        infer.addToModels = options.addToModels
        
        Predicates_ret, sences_ret = infer.infer(sentence, prac.action_cores['Filling'])
        java.shutdownJvm()
        
        Existing_words=[]
        Existing_roles=[]
        Existing_sences=[]
        for senc in sences_ret:
            for r in senc:
                temp_r1=r.partition(':')
                Existing_sences.append(temp_r1[0].title())

        for pred in Predicates_ret:
            for e in pred:
                temp_e1=e.partition('(')
                temp_e2=temp_e1[2].partition(',')
                temp_e3=temp_e2[0].partition('-')
                Existing_words.append(temp_e3[0].title())
                temp_e4=temp_e2[2].partition(')')
                Existing_roles.append(temp_e4[0].title())

    Prolog_mapping= {'Milk':  '0.025-0', 'Water': '0.005-0' ,'Oil': '0.04-0' , 'Chocolate' : '0.03-2'}
    Reverse_mapping={}
    for i in range(0, len(Existing_roles)):
        if Existing_roles[i] == 'Source':
            Reverse_mapping['mug']=Existing_words[i]
        if Existing_roles[i] == 'Goal':
            Reverse_mapping['pancake_maker']=Existing_words[i]
        if Existing_roles[i] == 'Theme':
            Reverse_mapping['liquid']=Existing_words[i]
#initialization
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
    init_icra=0;
    #check if icra experiment is loaded
    query = prolog.query("simulator_value(bbox(mug,[MINX,MINY,_,MAXX,MAXY,_]), 25.9, '0.005-0')")
    for solution in query.solutions(): 
        temp=solution['MINX']
        if  temp == 0.858284 :
            init_icra=1
    query.finish()
    #if not loaded than load
    if init_icra == 0 :
        query = prolog.query("findall(_,load_icra13_exp1,_)")
        query.finish()
    #select timeline
    
    TL="'"+Prolog_mapping[Reverse_mapping['liquid']]+"'"
    time_over=[]
    time_tilted=[]
    time_in_mug_full=[]
    time_in_mug_spill=[]
    time_on_pancakemaker=[]
    print Existing_words
    print Existing_roles
    #query over condition
    qer="holds_tt(over('mug/mug_body',pancake_maker,0.7), T, "+TL+"),[Begin,End]=T"
    query = prolog.query(qer)
    for solution in query.solutions():
        time_over.append(solution['Begin'])
        time_over.append(solution['End'])
    query.finish()
    #query tilted condition
    qer="holds_tt(tilted('mug/mug_body',[0,0,0],[pi/12,0.0,0.0]), T, "+TL+"),[Begin,End]=T"
    query = prolog.query(qer)
    for solution in query.solutions():
        time_tilted.append(solution['Begin'])
        time_tilted.append(solution['End'])
    query.finish()
    #query pancake_mix in mug - get end time of full mug
    qer="findall(T, (holds(in_cluster(A,mug), T, "+TL+"),simulator_value(num_particles(A,[N]),T,"+TL+"), N>=30), T_int),nth0(0,T_int,Begin),reverse(T_int,[End| _])"
    query = prolog.query(qer)
    for solution in query.solutions():
        time_in_mug_full.append(solution['Begin'])
        time_in_mug_full.append(solution['End'])
    query.finish()
    time_in_mug_full[0]=time_in_mug_full[0]-0.1
    #find time when liquis is poured from mug
    qer="findall(T, (holds(in_cluster(A,mug), T, "+TL+"),simulator_value(num_particles(A,[N]),T, "+TL+"), N<30, N>=0), T_int),nth0(0,T_int,Begin),reverse(T_int,[End| _])"
    query = prolog.query(qer)
    for solution in query.solutions():
        time_in_mug_spill.append(solution['Begin'])
        time_in_mug_spill.append(solution['End'])
    query.finish()
    #find time when liquid is on pancake_maker
    qer="findall(T, (holds(on_cluster(A,pancake_maker), T, "+TL+")), T_int),nth0(0,T_int,Begin),reverse(T_int,[End| _])"
    query = prolog.query(qer)
    for solution in query.solutions():
        time_on_pancakemaker.append(solution['Begin'])
        time_on_pancakemaker.append(solution['End'])
    query.finish()
    time_on_pancakemaker[1]=time_on_pancakemaker[1]+10
    time_seq= [time_over[0], time_over[1],time_tilted[0],time_tilted[1], time_in_mug_full[0], time_on_pancakemaker[0],time_on_pancakemaker[1]]
    time_seq_sort=sorted(time_seq)
    my_db=[]
    os.chdir('/home/kenji/ros_sr/mturk/spatial_relations')
    for i in range(0,len(time_seq_sort)):
        my_db.append(open("db/"+Reverse_mapping['mug']+"_"+Reverse_mapping['liquid']+"_"+Reverse_mapping['pancake_maker']+"_Stage"+str(i)+".db", 'w'))
        for k in range(0,len(Existing_words)):
            print >> my_db[i], "has_role"+'('+Existing_words[k]+','+Existing_roles[k]+')'
        #for k in range(0,len(Existing_objects)):
            #print >> my_db[i], 'hasSense('+Existing_objects[k]+', Sense'+str(k)+')'
           # print >> my_db[i], 'isa(Sense'+ str() +', '+ Object_synsets[k]+')'
    for i in range(0,len(time_seq_sort)):
        if (i == 0):
            print >> my_db[i], 'begin_state(YES)' 
        if (i == len(time_seq_sort) -1):
            print >> my_db[i], 'end_state(YES)'
            
        if (time_seq_sort[i] >=time_over[0] and  time_seq_sort[i] <=time_over[1]):
            print >> my_db[i], "over("+Reverse_mapping['mug']+","+Reverse_mapping['pancake_maker']+")"
            if (i != 0): print >> my_db[i-1], "over_next("+Reverse_mapping['mug']+","+Reverse_mapping['pancake_maker']+")"
        if (time_seq_sort[i] >=time_tilted[0] and  time_seq_sort[i] <=time_tilted[1]):
            print >> my_db[i], "tilted("+Reverse_mapping['mug']+")"
            if (i != 0): print >> my_db[i-1], "tilted_next("+Reverse_mapping['mug']+")"
        if (time_seq_sort[i] >=time_in_mug_full[0] and  time_seq_sort[i] <=time_in_mug_full[1]):
            print >> my_db[i], "in("+Reverse_mapping['liquid']+","+Reverse_mapping['mug']+")"
            if (i != 0): print >> my_db[i-1], "in_next("+Reverse_mapping['liquid']+","+Reverse_mapping['mug']+")"
        if (time_seq_sort[i] >=time_in_mug_spill[0] and  time_seq_sort[i] <=time_in_mug_spill[1]):
            print >> my_db[i], "in("+Reverse_mapping['liquid']+","+Reverse_mapping['mug']+")"
            if (i != 0): print >> my_db[i-1], "in_next("+Reverse_mapping['liquid']+","+Reverse_mapping['mug']+")"
        if (time_seq_sort[i] >=time_on_pancakemaker[0] and  time_seq_sort[i] <=time_on_pancakemaker[1]):
            print >> my_db[i], "in("+Reverse_mapping['liquid']+","+Reverse_mapping['pancake_maker']+")"
            if (i != 0): print >> my_db[i-1], "in_next("+Reverse_mapping['liquid']+","+Reverse_mapping['pancake_maker']+")"
    for i in range(0,len(time_seq_sort)):
        my_db[i].close()