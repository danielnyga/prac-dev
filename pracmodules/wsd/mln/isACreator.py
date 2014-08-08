from mln.database import readDBFromFile, Database
from mln.mln import readMLNFromFile
from mln.util import mergeDomains
from prac.core import PRAC
from prac.wordnet import WordNet
import sys

def workUpDb(db,f,concepts,fol=True):
        query = 'has_sense(?w,?s)'
        for result in db.query(query):
            word = result['?s']
            if(fol):
                db.addGroundAtom('is_a('+word+","+word+')')
            else:
                for concept in concepts:
                    db.addGroundAtom('is_a('+word+","+concept+')',prac.wordnet.wup_similarity(word,concept))
        db.write(f)

if __name__ == '__main__':
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)
    senses = prac.getModuleByName('wn_senses')
    senses.initialize()
    fol = True
    
    args = sys.argv[1:]
    if(args[0] == '--fuzzy'):
        fol = False;
        mlnPath = args[1]
        dbPath = args[2]
    else:
        mlnPath = args[0]
        dbPath = args[1]
    
    mln = readMLNFromFile(mlnPath)
    dbs = readDBFromFile(mln,dbPath)
    
    domains = map(lambda db: db.domains,dbs)
    dictDomains = mergeDomains(*domains)
    if(fol):
        f = open('fol_training.db','w+')
    else:
        f = open('fuzzy_training.db','w+')
    for db in dbs[:-1]:
        workUpDb(db,f,dictDomains['sense'],fol)
        f.write('---\n')
    workUpDb(dbs[-1],f,dictDomains['sense'],fol)
    f.close()    