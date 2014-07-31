from mln.database import readDBFromFile, Database
from mln.mln import readMLNFromFile
from mln.util import mergeDomains
from prac.core import PRAC
from prac.wordnet import WordNet
import sys

def workUpDb(db,f,concepts):
        query = 'has_sense(?w,?s)'
        for result in db.query(query):
            word = result['?s']
            #TODO add parameter which defines which is_a concepts should be created
            ###FUZZY###
            #for concept in concepts:
                #db.addGroundAtom('is_a('+word+","+concept+')',prac.wordnet.wup_similarity(word,concept))
            ###FOL###
            db.addGroundAtom('is_a('+word+","+word+')')
        db.write(f)

if __name__ == '__main__':
    args = sys.argv[1:]
    mlnPath = args[0]
    dbPath = args[1]
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)
    senses = prac.getModuleByName('wn_senses')
    senses.initialize()
    mln = readMLNFromFile(mlnPath)
    dbs = readDBFromFile(mln,dbPath)
    
    domains = map(lambda db: db.domains,dbs)
    dictDomains = mergeDomains(*domains)
    f = open('out.db','w+')
    for db in dbs[:-1]:
        workUpDb(db,f,dictDomains['sense'])
        f.write('---\n')
    workUpDb(dbs[-1],f,dictDomains['sense'])
    f.close()    