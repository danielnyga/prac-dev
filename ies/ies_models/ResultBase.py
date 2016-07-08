'''
Created on Sep 6, 2015

@author: seba
'''

class ResultBase(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self.num_nouns = 0
        self.num_verbs = 0
        self.num_adjs = 0
        self.num_unk = 0
        self.num_assert_senses = 0