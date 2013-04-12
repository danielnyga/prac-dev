import roslib; roslib.load_manifest('json_prolog')
import os
import rospy
import sys
import json_prolog
import java
import jpype
from grammar import parsePracFormula
from actioncore import PRAC
from actioncore.inference import *

class DatabaseGenerator(PRACReasoner):
    def __init__(self, action):
        self.action_type=action
    def run(self):
        if self.action_type == 'Filling':
            self.FillingGeneration()
        elif self.action_type == 'Flipping':
            self.FlippingGeneration()
        
    def FillingGeneration(self):
        action_type=self.action_type
        Existing_words=[]
        Existing_roles=[]
        db = self.pracinference.databases['core']
        for s in db.query('action_role(?w, ?r)'):
            tem_w=s['?w'].partition('-')
            tem_r=s['?r'].partition('-')
            Existing_words.append(tem_w[0].title())
            Existing_roles.append(tem_r[0].title())
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
        os.chdir('models/'+action_type+'/spatial_relations')
        for i in range(0,len(time_seq_sort)):
            my_db.append(open("db/"+Reverse_mapping['mug']+"_"+Reverse_mapping['liquid']+"_"+Reverse_mapping['pancake_maker']+"_Stage"+str(i)+".db", 'w'))
            for k in range(0,len(Existing_words)):
                print >> my_db[i], "has_role"+'('+Existing_words[k]+','+Existing_roles[k]+')'
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

    def FlippingGeneration(self):
            action_type=self.action_type
            Existing_words=[]
            Existing_roles=[]
            db = self.pracinference.databases['core']
            for s in db.query('action_role(?w, ?r)'):
                tem_w=s['?w'].partition('-')
                tem_r=s['?r'].partition('-')
                Existing_words.append(tem_w[0].title())
                Existing_roles.append(tem_r[0].title())
            Prolog_mapping= {'Pancake':  'f1'}
            Reverse_mapping={}
            Existing_words.append('Pan')
            Existing_roles.append('FixedLocation')
            for i in range(0, len(Existing_roles)):
                if Existing_roles[i] == 'Instrument':
                    Reverse_mapping['spatula']=Existing_words[i]
                if Existing_roles[i] == 'FixedLocation':
                    Reverse_mapping['pancake_maker']=Existing_words[i]
                if Existing_roles[i] == 'Theme':
                    Reverse_mapping['pancake']=Existing_words[i]
        #initialization
            rospy.init_node('test_json_prolog')
            prolog = json_prolog.Prolog()
            init_icra=0;
            #check if icra experiment is loaded
            query = prolog.query("holds_tt(contacts(pancake,pancake_maker),TI,f1),[Beg,En]=TI")
            for solution in query.solutions(): 
                temp=solution['Beg']
                if  temp == 31 :
                    init_icra=1
            query.finish()
            #if not loaded than load
            if init_icra == 0 :
                query = prolog.query("load_data_glove_flip")
                query.finish()
            #select timeline
            TL="'"+Prolog_mapping[Reverse_mapping['pancake']]+"'"
            time_cont_pan_maker=[]
            time_cont_pan_spat=[]
            time_on_spatula=[]
            time_tilted=[]
            #query contact between pancake and pancake_mixer condition
            qer="holds_tt(contacts(pancake,pancake_maker),TI,"+TL+"),[Begin,End]=TI"
            query = prolog.query(qer)
            for solution in query.solutions():
                time_cont_pan_maker.append(solution['Begin'])
                time_cont_pan_maker.append(solution['End'])
            query.finish()
        
            #query contact between pancake and spatula condition
            qer="holds_tt(contacts(pancake,spatula),TI,"+TL+"),[Begin,End]=TI"
            query = prolog.query(qer)
            for solution in query.solutions():
                time_cont_pan_spat.append(solution['Begin'])
                time_cont_pan_spat.append(solution['End'])
            query.finish()
        
            #query pancake on spatula
            qer="findall(TI,holds_tt(on(spatula,pancake),TI,"+TL+"),T_int),sort(T_int,[ [Begin | _ ] | _]),reverse(T_int,[[_,End|_] | _])"
            query = prolog.query(qer)
            for solution in query.solutions():
                time_on_spatula.append(solution['Begin'])
                time_on_spatula.append(solution['End'])
            query.finish()
            
            #query pancake tilted
            qer="findall(T,holds(tilted('pancake/pancake_body_1_3',[0,0,0],[pi/4.6,0.0,0.0]), T, "+TL+"),T_int),nth0(0,T_int,Begin),reverse(T_int,[End| _])"
            query = prolog.query(qer)
            for solution in query.solutions():
                time_tilted.append(solution['Begin'])
                time_tilted.append(solution['End'])
            query.finish()
            time_tilted[1]=time_tilted[1]+1
            time_seq= [time_cont_pan_maker[0], time_cont_pan_spat[0], time_tilted[0], time_on_spatula[0], time_tilted[1],time_cont_pan_maker[3]]
            time_seq_sort=sorted(time_seq)
            my_db=[]
            os.chdir('models/'+action_type+'/spatial_relations')
            for i in range(0,len(time_seq_sort)):
                my_db.append(open("db/"+Reverse_mapping['spatula']+"_"+Reverse_mapping['pancake']+"_Stage"+str(i)+".db", 'w'))
                for k in range(0,len(Existing_words)):
                    print >> my_db[i], "has_role"+'('+Existing_words[k]+','+Existing_roles[k]+')'
            for i in range(0,len(time_seq_sort)):
                if (i == 0):
                    print >> my_db[i], 'begin_state(YES)' 
                if (i == len(time_seq_sort) -1):
                    print >> my_db[i], 'end_state(YES)'
                if ((time_seq_sort[i] >=time_cont_pan_maker[0] and  time_seq_sort[i] <=time_cont_pan_maker[1]) or (time_seq_sort[i] >=time_cont_pan_maker[2] and  time_seq_sort[i] <=time_cont_pan_maker[3])):
                    print >> my_db[i], "contact("+Reverse_mapping['pancake']+","+Reverse_mapping['pancake_maker']+")"
                    if (i != 0): print >> my_db[i-1], "contact_next("+Reverse_mapping['pancake']+","+Reverse_mapping['pancake_maker']+")"
                if (time_seq_sort[i] >=time_cont_pan_spat[0] and  time_seq_sort[i] <=time_cont_pan_spat[1]):
                    print >> my_db[i], "contact("+Reverse_mapping['pancake']+","+Reverse_mapping['spatula']+")"
                    if (i != 0): print >> my_db[i-1], "contact_next("+Reverse_mapping['pancake']+","+Reverse_mapping['spatula']+")"
                if (time_seq_sort[i] >=time_on_spatula[0] and  time_seq_sort[i] <=time_on_spatula[1]):
                    print >> my_db[i], "on("+Reverse_mapping['pancake']+","+Reverse_mapping['spatula']+")"
                    if (i != 0): print >> my_db[i-1], "on_next("+Reverse_mapping['pancake']+","+Reverse_mapping['spatula']+")"
                if (time_seq_sort[i] >=time_tilted[0] and  time_seq_sort[i] <=time_tilted[1]):
                    print >> my_db[i], "tilted("+Reverse_mapping['pancake']+")"
                    if (i != 0): print >> my_db[i-1], "tilted_next("+Reverse_mapping['pancake']+")"
            for i in range(0,len(time_seq_sort)):
                my_db[i].close()