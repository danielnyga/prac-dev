'''
Created on June 7, 2016

@author: picklum
'''


class ConceptAlreadyExistsError(Exception): pass
class NoRationalNumberError(Exception): pass
class ParserError(Exception):
    def __init__(self, *args, **margs):
        Exception.__init__(self, *args, **margs)