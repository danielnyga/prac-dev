'''
Created on Sep 3, 2015

@author: seba
'''

def split_list_in_n_equal_sub_lists(l,n):
    dividor = len(l)/n
    result = []
    step =  0
    for i in range(1,n):
        result.append(l[step:dividor*i])
        step = dividor*i
    result.append(l[step:])
  
    return result