import os

import thread

from pracmln.mln.util import colorize
from pracmln.utils.visualization import DECLARATIONS
from pracmln.utils.latexmath2png import math2png


def partition(l, s):
    '''
    Partitions the list ``l`` into sublists of size ``s`` and returns a generator
    iterating over them.
    '''
    for i in range(0, len(l), s): yield l[i:i+s]


def prac_heading(s, upper=True, color='green'):
    '''
    Returns a colorized and formatted string for pretty priting module
    headings.

    :param s:       the string to be formatted
    :param upper:   (bool) if string should be converted to uppercase. default
                    is true
    :param color:   the color in which the heading should be printed. default
                    is green
    :return:        the colorized and formatted string
    '''
    b = colorize('+{}+'.format(''.ljust(len(s)+2, '=')), (None, color, True), True)
    t = colorize('| {} |'.format(s.upper() if upper else s), (None, color, True), True)
    return '\n{}\n{}\n{}\n'.format(b, t, b)


def synchronized(lock):
    '''
    Synchronization decorator.
    '''

    def wrap(f):
        def func(*args, **kw):
            lock.acquire()
            try:
                return f(*args, **kw)
            finally:
                lock.release()
        return func
    return wrap


def get_query_png(queries, dbs, filename='cond_prob', filedir='/tmp', skolemword=''):
    '''
    Preprocessing of png generation: assemble latex code for argmax term

    :param queries:     list or comma-separated string of query predicates
    :param dbs:         evidence database
    :param filename:    filename prefix of the generated file
    :param filedir:     location of temporary generated file
    :param skolemword:  string value for skolemword looked up in mongo database
    :return:            a png string generated by math2png
    '''
    safefilename = '{}-{}-{}'.format(filename, os.getpid(), thread.get_ident())
    declarations = DECLARATIONS + [r'''\newcommand{\simil}[1]{\ensuremath{sim\left(\begin{array}{cc}#1\end{array}\right)}}''']

    if isinstance(queries, str):
        queries = queries.split(',')

    evidencelist = []
    if isinstance(dbs, list):
        for db in dbs:
            evidencelist.extend([e for e in db.evidence.keys() if db.evidence[e] == 1.0])
    elif isinstance(dbs, str):
        evidencelist = dbs.split(',')
    else:
        evidencelist.extend([e if dbs.evidence[e] == 1.0 else '!' + e for e in dbs.evidence.keys()])

    # escape possibly occurring underscores in predicate names
    query = r'''\\'''.join([r'''\text{{ {0} }} '''.format(q.replace('_', '\_')) for q in queries])
    evidence = r'''\\'''.join([r'''\text{{ {0} }} '''.format(e.replace('_', '\_')) for e in evidencelist])

    head = '\\prod_{{ {0} }}'.format(query)
    underset = '_{{ \\tiny\\textit{{adt}} \in \\textit{{ADT-lib}} }}'
    query = r'''\text{{ adt, }} '''
    skolem = '\\text{{ {} }}: '.format(skolemword)

    # generate actual equation
    head = r'''{0}\argmax{1}'''.format(head, underset)
    bracket_term = r'''\simil{{ \begin{{array}}{{c}}{0}\end{{array}} & {1}\begin{{array}}{{c}}{2}\end{{array}} }}'''.format(
        query, skolem, evidence)
    eq = r'''{} {}'''.format(head, bracket_term)

    return math2png(eq, filedir, declarations=declarations, filename=safefilename, size=10)


#===============================================================================
# main function for testing only!
#===============================================================================

if __name__ == '__main__':
    print list(partition(range(2), 3))
    