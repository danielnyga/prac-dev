'''
Created on Nov 27, 2012

@author: nyga
'''

'''
in order to run this script:
    add '/opt/ros/fuerte/lib/python2.7/dist-packages to your PYTHONPATH
    start roscore
    $ rosrun json_prolog json_prolog slope_data_analysis
'''


import roslib
roslib.load_manifest('json_prolog')
import rospy
import json_prolog
import math

mapping = {'pancake': 'pancake.n.01',
           'spatula': 'spatula.n.01',
           'ias_kitchen': 'kitchen_table.n.01',
           'pancake_maker': 'pan.n.01'}

def initPARMA(prolog):
    query = prolog.query("create_timeline(flip1,['flip1-contacts.pl'])")
    for s in  query.solutions():
        print s
    query.finish()
    query = prolog.query("create_timeline_stats(flip1)")
    for s in query.solutions():
        print s
    query.finish()
    

if __name__ == '__main__':
    nodeName = 'PRAC2PARMA'
    rospy.init_node(nodeName, anonymous=True)
    prolog = json_prolog.Prolog()
#    initPARMA(prolog)
    q = prolog.query('load_data_glove_flip')
    for s in q.solutions(): pass
    q.finish()
    
    # Retrieve all spatial relations for each "frame"
    timeline = 'f1'
    q = prolog.query('tl_begin(%s, BEGIN), tl_end(%s,END)' % (timeline, timeline))
    frameList = None
    for s in q.solutions():
        start = s.get('BEGIN', None)
        end = s.get('END')
        frameList = range(int(math.ceil(start)), int(math.floor(end)))
        print frameList
    q.finish()
    
    relations = []
    previous = None
    for frame in frameList:
        relations = set()
#        print 'contact:'
        q = prolog.query("holds(contacts(O1,O2), %s, %s)" % (frame, timeline))
        for s in q.solutions():
            if not s['O1'] in mapping.keys() or not s['O2'] in mapping.keys():
                continue 
            rel = 'contact(%s, %s)' % (mapping[s['O1']], mapping[s['O2']])
            relations.add(rel)
        q.finish()
#        print 'on:'
        q = prolog.query("holds(on(O1,O2), %s, %s)" % (frame, timeline))
        for s in q.solutions():
            if not s['O1'] in mapping.keys() or not s['O2'] in mapping.keys():
                continue 
            rel = 'on(%s, %s)' % (mapping[s['O1']], mapping[s['O2']])
            relations.add(rel)
        q.finish()
#        print 'over:'
        q = prolog.query("holds(over(O1,O2,0.1), %s, %s)" % (frame, timeline))
        for s in q.solutions():
            if not s['O1'] in mapping.keys() or not s['O2'] in mapping.keys():
                continue 
            rel = 'over(%s, %s)' % (mapping[s['O1']], mapping[s['O2']])
            relations.add(rel)
        q.finish()
        if previous is None or not previous == relations:
            print frame
            previous = relations
            for r in relations:
                print r
            print '********************'
#    query = prolog.query("tl_begin(flip1,BEGIN), tl_end(flip1,END), numlist(BEGIN,END,FRAMES), member(F,FRAMES), setof(contacts(O1,O2),  holds(contacts(O1,O2), F, flip1), CONTACT), setof(on(O3,O4),  holds(on(O3,O4), F, flip1), ON), setof(over(O5,O6),  holds(over(O5,O6), F, flip1), OVER)")
#    on = prolog.query("tl_begin(flip1,BEGIN), tl_end(flip1,END), numlist(BEGIN,END,FRAMES), member(F,FRAMES), setof(on(O1,O2),  holds(on(O1,O2), F, flip1), L)")
#    over = prolog.query("tl_begin(flip1,BEGIN), tl_end(flip1,END), numlist(BEGIN,END,FRAMES), member(F,FRAMES), setof(over(O1,O2),  holds(over(O1,O2), F, flip1), L)")
    
#    for query in [contacts, on, over]:
#    for s in query.solutions():
#        print s
#        relations = s.get('L', None)
#        print s.get('FRAME', None)
#        if relations is not None:
#            for r in relations:
#                print '\t', r
#    contacts.finish()
#    on.finish()
#    over.finish()
#    query.finish()
        
    
    