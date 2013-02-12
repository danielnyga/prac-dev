'''
Created on Feb 13, 2012

@author: nyga
'''

from pyparsing import *
import re

class RBMParser(object):
    # Literals
    dot = Literal('.')
    e = CaselessLiteral('E')
    arg = Combine(Optional('+') + Word(alphas))
    tilde = Literal('~').suppress()
    bl = Literal('(').suppress()
    br = Literal(')').suppress()
    comma = Literal(',').suppress()
    plus = Literal('+')
    underscore = Literal('_')
    bracel = Literal('{').suppress()
    bracer = Literal('}').suppress()
    equals = Literal('=').suppress()
    comkey = Literal('//')
    excl = Literal('!')
    minus = Literal('-').suppress()

    # Floating point number
    fnumber = Combine( Word( "+-"+nums, nums ) + Optional( dot + Optional( Word( nums ) ) ) +
                           Optional( e + Word( "+-"+nums, nums ) ) )
    
    symbol = Word(alphanums + '_-')
    
    # comment
    comment = Optional(comkey + ZeroOrMore(Word(printables))).suppress()

    # argument and (comma-separated) argument lists 
    arg = Combine(Optional(plus) + symbol)
    argKey = Combine(Optional(':') + symbol)
    arglist = arg + ZeroOrMore(comma + arg)
    arglistDecl = argKey + ZeroOrMore(comma + argKey)

    # predicates
    predicate = Optional(excl) + symbol + bl + arglist + br
    predicateDecl = symbol + bl + arglistDecl# + comment

    #formulas
    formula = fnumber + tilde + fnumber + predicate + ZeroOrMore(tilde + fnumber + predicate) + comment
    formulaDecl = predicate + ZeroOrMore(tilde + predicate) + comment

    # domain
    domain = symbol + equals + bracel + symbol + ZeroOrMore(comma + symbol) + bracer# + comment
    
    @staticmethod
    def parsePredicateDeclaration(s):
        return RBMParser.predicateDecl.parseString(s)
    
    @staticmethod
    def parseDomain(s):
        return RBMParser.domain.parseString(s)
    
    @staticmethod
    def parsePredicate(s):
        return RBMParser.predicate.parseString(s)

    @staticmethod
    def parseFormulaDecl(s):
        return RBMParser.formulaDecl.parseString(s)

if __name__ == '__main__':
    
    f = '1.2 ~ -1e-6 hasSense(Constant) ~ 3.21 hasRole(bla,+b1)\n'
    decl = 'hasROle(:sense, key, :arg)'
    p = '!hasSense(+arg1,arg2,arg3)'
    a = "S3990474_pot"
    print RBMParser.arg.parseString(a)
    d = 'senses = {S3990474_pot,S7936548_soda_water,S14845743_water,S2881193_bowl}'
    fd = 'hasSense(w1,Water,s) ~ hasRole(w1,Theme,s)'
#    print RBMParser.formula.parseString(f)
#    print RBMParser.parsePredicateDeclaration(decl)
    print RBMParser.parseDomain(d)
#    print RBMParser.parsePredicate(p)
#    print RBMParser.parseFormulaDecl(fd)
    
    if re.match(r'\w*', 'kdsjfnlaksn\n'):
        print 'shit'
    
    s = 'bblablabla !// \n'
    try: s = s[:s.index('//')] # remove comments
    except: pass
    print s        