from prac.core import PRAC
from prac.wordnet import WordNet
from mln.mln import readMLNFromFile
from mln.database import readDBFromFile, Database
import sys
def workUpDb(db,f):
        myList = list(db.query(query))
        for result in myList:
            word = result['?s']
            for res in myList:
                db.addGroundAtom('is_a('+word+","+res['?s']+')',prac.wordnet.wup_similarity(word,res['?s']))
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
    query = 'has_sense(?w,?s)'
    i = -1;
    f = open("out.db", 'w+')
    for db in dbs[:-1]:
        workUpDb(db, f)
        f.write('---\n')
    lastDb = dbs[-1]
    workUpDb(lastDb,f)
    f.close();            