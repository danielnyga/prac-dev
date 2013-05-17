from pyparser.driver import Driver
from pyparser.grammar2x import Grammar

g = Grammar()

def suite(text):
    d = Driver(g)
    return d.parse_string(text)

# dummy
def st2tuple(tree, line_info=1):
    return tree
    
