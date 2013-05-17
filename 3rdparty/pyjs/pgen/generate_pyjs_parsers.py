"""
Tool to generate parser suitable for pyjs translator.

Produces grammar2x.py, symbol.py

Should also produce token.py

"""
from string import Template
import datetime
import sys
from lib2to3.compiler import parser
from lib2to3.pgen2.driver import load_grammar, grammar as grammar_module

gen_date = datetime.datetime.now()
python_version = sys.version.split('\n')[0]

print "Generating grammar2x.py"

opmap = grammar_module.opmap
g = load_grammar('grammar2x.txt', force=True)
#g3 = load_grammar('grammar3x.txt', force=True)
g_templ = Template(open('grammar2x.py.templ').read())
attrs_assign = []
attrs = ['symbol2number', 'number2symbol', 'states', 'dfas', 'labels', 'keywords', 'tokens', 'symbol2label', 'start']

for attr in attrs:
    attrs_assign.append(''.join([attr, " = ", repr(getattr(g, attr))]))

attrs_assign_str = ("\n" + " "*4).join(attrs_assign)

opmap_assign = "opmap = {0!r}".format(opmap)

out = g_templ.substitute(gen_date=gen_date,
                         python_version=python_version,
                         grammar_attrs_assign=attrs_assign_str, 
                         opmap_assign=opmap_assign)

out_f = open('grammar2x.py', 'w')
out_f.write(out)
out_f.close()

print "Generating symbol.py"

symb_templ = Template(open('symbol.py.templ').read())
symbols_assign = []

for name, symbol in g.symbol2number.iteritems():
    symbols_assign.append(name + " = " + repr(symbol))

symbols_assign_str = "\n".join(symbols_assign)
out_f = open("symbol.py", 'w')
out_f.write(symb_templ.substitute(gen_date=gen_date,
                                  python_version=python_version,
                                  symbols_assign=symbols_assign_str))
out_f.close()
