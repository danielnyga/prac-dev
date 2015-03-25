from mln.database import readDBFromFile, Database
from mln.mln import readMLNFromFile
from mln.util import mergeDomains
from prac.core import PRAC
from prac.wordnet import WordNet
import sys

def workUpDb(db,concepts,queryPredicate,conceptVariable,prac,fol=True):
        db_ = db.duplicate()
        query = queryPredicate
        for result in db_.query(query):
            word = result[conceptVariable]
            if(fol):
                db_.addGroundAtom('is_a('+word+","+word+')')
            else:
                for concept in concepts:
                    db_.addGroundAtom('is_a('+word+","+concept+')',prac.wordnet.wup_similarity(word,concept))
        return db_

def createIsAEvidence(mln,dbs,queryDomain,queryPredicate,conceptVariable,fol=True):
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)
    senses = prac.getModuleByName('wn_senses')
    senses.initialize()
    
    domains = map(lambda db: db.domains,dbs)
    dictDomains = mergeDomains(*domains)
    dbs_ = []
    
    for db in dbs:
        dbs_.append(workUpDb(db,dictDomains[queryDomain],queryPredicate,conceptVariable,prac,fol))
    
    return dbs_

if __name__ == '__main__':
    args = sys.argv[1:]
    if(args[0] == '--fuzzy'):
        fol = False;
        mlnPath = args[1]
        dbPath = args[2]
        queryDomain = args[3]
        queryPredicate = args[4]
        conceptVariable = args[5]
    else:
        fol = True
        mlnPath = args[0]
        dbPath = args[1]
        queryDomain = args[2]
        queryPredicate = args[3]
        conceptVariable = args[4]
    
    mln = readMLNFromFile(mlnPath)
    dbs = readDBFromFile(mln,dbPath)
    
    
    if(fol):
        f = open('fol_training.db','w+')
    else:
        f = open('fuzzy_training.db','w+')
    
    dbs_ = createIsAEvidence(mln, dbs, queryDomain, queryPredicate, conceptVariable, fol)    
    
    for db in dbs_[:-1]:
        db.write(f)
        f.write('---\n')
    dbs_[-1].write(f)
    f.close()    