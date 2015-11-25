# Natural Language Toolkit: Relation Extraction
#
# Copyright (C) 2001-2010 NLTK Project
# Author: Ewan Klein <ewan@inf.ed.ac.uk>
# URL: <http://www.nltk.org/>
# For license information, see LICENSE.TXT

"""
Code for extracting relational triples from the ieer and conll2002 corpora.

Relations are stored internally as dictionaries ('reldicts'). 

The two serialization outputs are I{rtuple} and I{clause}. 
   - An I{rtuple} is a tuple of the form C{(subj, filler, obj)}, 
     where C{subj} and C{obj} are pairs of Named Entity mentions, and C{filler} is the string of words   
     occurring between C{sub} and C{obj} (with no intervening NEs). Strings are printed via C{repr()} to
     circumvent locale variations in rendering utf-8 encoded strings.
   - A I{clause} is an atom of the form C{relsym(subjsym, objsym)}, 
     where the relation, subject and object have been canonicalized to single strings.

"""

# todo: get a more general solution to canonicalized symbols for clauses -- maybe use xmlcharrefs?

from prac_nltk.compat import defaultdict

from string import join
import re
import htmlentitydefs
from itertools import ifilter

# Dictionary that associates corpora with NE classes
NE_CLASSES = {
    'ieer': ['LOCATION', 'ORGANIZATION', 'PERSON', 'DURATION', 
            'DATE', 'CARDINAL', 'PERCENT', 'MONEY', 'MEASURE'],
    'conll2002': ['LOC', 'PER', 'ORG'],
    'ace': ['LOCATION', 'ORGANIZATION', 'PERSON', 'DURATION', 
            'DATE', 'CARDINAL', 'PERCENT', 'MONEY', 'MEASURE', 'FACILITY', 'GPE'],
    }

# Allow abbreviated class labels                   
short2long = dict(LOC = 'LOCATION', ORG = 'ORGANIZATION', PER = 'PERSON')
long2short = dict(LOCATION ='LOC', ORGANIZATION = 'ORG', PERSON = 'PER')


def _expand(type):
    """
    Expand an NE class name.
    @type type: C{str}
    @rtype: C{str}
    """
    try:
        return short2long[type]
    except KeyError:
        return type
    
def class_abbrev(type):
    """
    Abbreviate an NE class name.
    @type type: C{str}
    @rtype: C{str}
    """
    try:
        return long2short[type]
    except KeyError:
        return type
    
    
def _join(lst, sep=' ', untag=False):
    """
    Join a list into a string, turning tags tuples into tag strings or just words.
    @param untag: if C{True}, omit the tag from tagged input strings.
    @type lst: C{list}
    @rtype: C{str}
    """
    try:
        return join(lst, sep=sep)
    except TypeError:
        if untag:
            return join([tup[0] for tup in lst], sep=sep)            
        from prac_nltk.tag import tuple2str
        return join([tuple2str(tup) for tup in lst], sep=sep) 

def descape_entity(m, defs=htmlentitydefs.entitydefs):
    """
    Translate one entity to its ISO Latin value.
    Inspired by example from effbot.org
    

    """
    #s = 'mcglashan_&amp;_sarrail'
    #l = ['mcglashan', '&amp;', 'sarrail']
    #pattern = re.compile("&(\w+?);")
    #new = list2sym(l)
    #s = pattern.sub(descape_entity, s)
    #print s, new
    try:
        return defs[m.group(1)]
    
    except KeyError:
        return m.group(0) # use as is    

def list2sym(lst):
    """
    Convert a list of strings into a canonical symbol.
    @type lst: C{list}
    @return: a Unicode string without whitespace
    @rtype: C{unicode}
    """
    sym = _join(lst, '_', untag=True)
    sym = sym.lower()
    ENT = re.compile("&(\w+?);")
    sym = ENT.sub(descape_entity, sym)
    sym = sym.replace('.', '')
    return sym

def mk_pairs(tree):
    """
    Group a chunk structure into a list of pairs of the form (list(str), L{Tree})
    
    In order to facilitate the construction of (L{Tree}, string, L{Tree}) triples, this
    identifies pairs whose first member is a list (possibly empty) of terminal
    strings, and whose second member is a L{Tree} of the form (NE_label, terminals).
    
    @param tree: a chunk tree
    @return: a list of pairs (list(C{str}), L{Tree})
    @rtype: C{list} of C{tuple}
    """

    from prac_nltk.tree import Tree
    
    pairs = []
    pair = [[], None]
    
    for dtr in tree:
        if not isinstance(dtr, Tree):
            pair[0].append(dtr)
        else:
            # dtr is a Tree
            pair[1] = dtr
            pairs.append(pair)
            pair = [[], None]
    return pairs

            
def mk_reldicts(pairs, window=5, trace=0):
    """
    Converts the pairs generated by L{mk_pairs} into a 'reldict': a dictionary which
    stores information about the subject and object NEs plus the filler between them.
    Additionally, a left and right context of length =< window are captured (within 
    a given input sentence).
    
    @param pairs: a pair of list(str) and L{Tree}, as generated by 
    @param window: a threshold for the number of items to include in the left and right context
    @type window: C{int}
    @return: 'relation' dictionaries whose keys are 'lcon', 'subjclass', 'subjtext', 'subjsym', 'filler', objclass', objtext', 'objsym' and 'rcon'
    @rtype: C{list} of C{defaultdict}
    """
    result = []
    while len(pairs) > 2:
        reldict = defaultdict(str)
        reldict['lcon'] = _join(pairs[0][0][-window:])
        reldict['subjclass'] = pairs[0][1].node
        reldict['subjtext'] = _join(pairs[0][1].leaves())
        reldict['subjsym'] = list2sym(pairs[0][1].leaves())
        reldict['filler'] = _join(pairs[1][0])
        reldict['objclass'] = pairs[1][1].node
        reldict['objtext'] = _join(pairs[1][1].leaves())
        reldict['objsym'] = list2sym(pairs[1][1].leaves())
        reldict['rcon'] = _join(pairs[2][0][:window])
        if trace:
            print "(rel(%s, %s)" % (reldict['subjclass'], reldict['objclass'])
        result.append(reldict)
        pairs = pairs[1:]
    return result

def extract_rels(subjclass, objclass, doc, corpus='ace', pattern=None, window=10):
    """
    Filter the output of L{mk_reldicts} according to specified NE classes and a filler pattern.
    
    The parameters C{subjclass} and C{objclass} can be used to restrict the 
    Named Entities to particular types (any of 'LOCATION', 'ORGANIZATION', 
    'PERSON', 'DURATION', 'DATE', 'CARDINAL', 'PERCENT', 'MONEY', 'MEASURE').

    @param subjclass: the class of the subject Named Entity.
    @type subjclass: C{string}
    @param objclass: the class of the object Named Entity.
    @type objclass: C{string}
    @param doc: input document
    @type doc: C{ieer} document or a list of chunk trees
    @param corpus: name of the corpus to take as input; possible values are         
    'ieer' and 'conll2002'
    @type corpus: C{string}
    @param pattern: a regular expression for filtering the fillers of
    retrieved triples.
    @type pattern: C{SRE_Pattern}
    @param window: filters out fillers which exceed this threshold
    @type window: C{int}
    @return: see L{mk_reldicts}
    @rtype: C{list} of C{defaultdict}
    """
    
    if subjclass and subjclass not in NE_CLASSES[corpus]:
        if _expand(subjclass) in NE_CLASSES[corpus]:
            subjclass = _expand(subjclass)
        else:
            raise ValueError, "your value for the subject type has not been recognized: %s" % subjclass
    if objclass and objclass not in NE_CLASSES[corpus]:
        if _expand(objclass) in NE_CLASSES[corpus]:
            objclass = _expand(objclass)
        else:
            raise ValueError, "your value for the object type has not been recognized: %s" % objclass
    if corpus == 'ace' or corpus == 'conll2002':
        pairs = mk_pairs(doc)
    elif corpus == 'ieer':
        pairs = mk_pairs(doc.text) + mk_pairs(doc.headline)
    else:            
        raise ValueError, "corpus type not recognized"
            
    reldicts = mk_reldicts(pairs)
    
    relfilter = lambda x: (x['subjclass'] == subjclass and
                           len(x['filler'].split()) <= window and 
                           pattern.match(x['filler']) and
                           x['objclass'] == objclass)
        
    return filter(relfilter, reldicts)
  

def show_raw_rtuple(reldict, lcon=False, rcon=False):
    """
    Pretty print the reldict as an rtuple.
    @param reldict: a relation dictionary
    @type reldict: C{defaultdict}
    """
    items = [class_abbrev(reldict['subjclass']), reldict['subjtext'], reldict['filler'], class_abbrev(reldict['objclass']), reldict['objtext']]
    format = '[%s: %r] %r [%s: %r]'
    if lcon:
        items = [reldict['lcon']] + items
        format = '...%r)' + format
    if rcon:
        items.append(reldict['rcon'])
        format = format + '(%r...'
    printargs = tuple(items)
    return format % printargs

def show_clause(reldict, relsym):
    """
    Print the relation in clausal form.
    @param reldict: a relation dictionary
    @type reldict: C{defaultdict}
    @param relsym: a label for the relation
    @type relsym: C{str}
    """
    items = (relsym, reldict['subjsym'], reldict['objsym'])
    return "%s(%r, %r)" % items


#######################################################
# Demos of relation extraction with regular expressions
#######################################################

############################################
# Example of in(ORG, LOC)
############################################
def in_demo(trace=0, sql=True):
    """
    Select pairs of organizations and locations whose mentions occur with an
    intervening occurrence of the preposition "in".

    If the sql parameter is set to True, then the entity pairs are loaded into
    an in-memory database, and subsequently pulled out using an SQL "SELECT"
    query.
    """
    from prac_nltk.corpus import ieer
    if sql:
        try:
            import sqlite3
            connection =  sqlite3.connect(":memory:")
            connection.text_factory = sqlite3.OptimizedUnicode
            cur = connection.cursor()
            cur.execute("""create table Locations
            (OrgName text, LocationName text, DocID text)""")
        except ImportError:
            import warnings
            warnings.warn("Cannot import sqlite; sql flag will be ignored.")
            
        
    IN = re.compile(r'.*\bin\b(?!\b.+ing)')
    
    print
    print "IEER: in(ORG, LOC) -- just the clauses:"
    print "=" * 45

    for file in ieer.fileids():
        for doc in ieer.parsed_docs(file):
            if trace:
                print doc.docno
                print "=" * 15
            for rel in extract_rels('ORG', 'LOC', doc, corpus='ieer', pattern=IN):
                print show_clause(rel, relsym='IN')
                if sql:
                    try:
                        rtuple = (rel['subjtext'], rel['objtext'], doc.docno)
                        cur.execute("""insert into Locations 
                                    values (?, ?, ?)""", rtuple)
                        connection.commit()
                    except NameError:
                        pass
                    
    if sql:
        try:           
            cur.execute("""select OrgName from Locations
                        where LocationName = 'Atlanta'""")
            print
            print "Extract data from SQL table: ORGs in Atlanta"
            print "-" * 15
            for row in cur:
                print row
        except NameError:
            pass


############################################
# Example of has_role(PER, LOC)
############################################
    
def roles_demo(trace=0):
    from prac_nltk.corpus import ieer
    roles = """
    (.*(                   # assorted roles
    analyst|
    chair(wo)?man|
    commissioner|
    counsel|
    director|
    economist|
    editor|
    executive|         
    foreman|
    governor|
    head|
    lawyer|
    leader|
    librarian).*)|
    manager|
    partner|
    president|
    producer|
    professor|
    researcher|
    spokes(wo)?man|
    writer|
    ,\sof\sthe?\s*  # "X, of (the) Y"
    """
    ROLES = re.compile(roles, re.VERBOSE)
    
    print
    print "IEER: has_role(PER, ORG) -- raw rtuples:"
    print "=" * 45
    
    for file in ieer.fileids():
        for doc in ieer.parsed_docs(file):
            lcon = rcon = False
            if trace:
                print doc.docno
                print "=" * 15
                lcon = rcon = True
            for rel in extract_rels('PER', 'ORG', doc, corpus='ieer', pattern=ROLES):
                print show_raw_rtuple(rel, lcon=lcon, rcon=rcon)

    
##############################################
### Show what's in the IEER Headlines
##############################################
    
   
def ieer_headlines():
    
    from prac_nltk.corpus import ieer
    from prac_nltk.tree import Tree
    
    print "IEER: First 20 Headlines"
    print "=" * 45
    
    trees = [doc.headline for file in ieer.fileids() for doc in ieer.parsed_docs(file)]
    for tree in trees[:20]:
        print
        print "%s:\n%s" % (doc.docno, tree)
 

        
#############################################
## Dutch CONLL2002: take_on_role(PER, ORG
#############################################
    
def conllned(trace=1):
    """
    Find the copula+'van' relation ('of') in the Dutch tagged training corpus
    from CoNLL 2002.
    """

    from prac_nltk.corpus import conll2002
        
    vnv = """
    (
    is/V|    # 3rd sing present and 
    was/V|   # past forms of the verb zijn ('be')
    werd/V|  # and also present 
    wordt/V  # past of worden ('become)
    )
    .*       # followed by anything
    van/Prep # followed by van ('of')
    """
    VAN = re.compile(vnv, re.VERBOSE)
    
    print
    print "Dutch CoNLL2002: van(PER, ORG) -- raw rtuples with context:"
    print "=" * 45
    
    
    for doc in conll2002.chunked_sents('ned.train'):
        lcon = rcon = False
        if trace:
                lcon = rcon = True
        for rel in extract_rels('PER', 'ORG', doc, corpus='conll2002', pattern=VAN, window=10):
            print show_raw_rtuple(rel, lcon=True, rcon=True)
    
#############################################
## Spanish CONLL2002: (PER, ORG)
#############################################

def conllesp():
    from prac_nltk.corpus import conll2002

    de = """
    .*
    (
    de/SP|
    del/SP
    )
    """
    DE = re.compile(de, re.VERBOSE)
    
    print
    print "Spanish CoNLL2002: de(ORG, LOC) -- just the first 10 clauses:"
    print "=" * 45
    rels = [rel for doc in conll2002.chunked_sents('esp.train')
            for rel in extract_rels('ORG', 'LOC', doc, corpus='conll2002', pattern = DE)]
    for r in rels[:10]: print show_clause(r, relsym='DE')
    print


def ne_chunked():
    IN = re.compile(r'.*\bin\b(?!\b.+ing)')
    rels = []
    for sent in nltk.corpus.treebank.tagged_sents()[:100]:
        sent = nltk.ne_chunk(sent)
        print extract_rels('ORG', 'LOC', sent, corpus='ace', pattern = IN)
        

if __name__ == '__main__':
    import prac_nltk as nltk
    from prac_nltk.sem import relextract
    in_demo(trace=0)
    roles_demo(trace=0)
    conllned()
    conllesp()
    ieer_headlines()
    






    
