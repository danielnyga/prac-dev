'''
Created on Jul 26, 2016

@author: nyga
'''
import json
import importlib


if __name__ == '__main__':
    #===========================================================================
    # Import the NL parsing module and get its CL parameter parser
    #===========================================================================
    nlmod = importlib.import_module('nlparse')
    parser = nlmod.parser
    options, args = parser.parse_args()
    nlmod.main([json.dumps(arg) for arg in args], options)
