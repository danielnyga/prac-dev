'''
Created on Nov 19, 2011

@author: nyga
'''

import sys

def combine(values):
    ''' This method returns all possible combinations of values.
        Values is a nested list of values.
    '''
    combinations = []
    combine_internal(values, [], combinations)
    return combinations

def combine_internal(values,t, combinations):
    if len(values) == 0:
        combinations.append(t)
        return t
    car = values[0]
    if len(values) > 1: cdr = values[1:] 
    else: cdr = []
    result = []
    for v in car:
        result.extend((combine_internal(cdr,(t + [v]),combinations)))
    return result

def get(dict,key):
    try:
        return dict[key]
    except:
        return None

def index(list,el):
    ''' This is a modified version of list.index(). It returns
        the index of the first occurrence of el, or -1, if there
        is no element el in list.
    '''
    try:
        return list.index(el)
    except:
        return -1
    
def showProgress(x):
    x = int(x * 100)
    sys.stdout.write('\r[' + '='*x + ' '*(99-x)+']')
    sys.stdout.flush()