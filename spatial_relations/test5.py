from wcsp.converter import WCSPConverter
from MLN.MarkovLogicNetwork import Database
from MLN.MarkovLogicNetwork import MLN


if __name__ == '__main__':
    name='tapwatcup.db'
    my_db=[]
    solut_atoms=[]
    finall_stage=False
    counter=0
    mln = MLN('./learned.mln')
    db = Database(mln, "./db_inference/"+name)

    queryPreds = ['on_next', 'in_next', 'over_next', 'tilted_next','begin_state','end_state']
    for pred in mln.predicates:
        if not pred in queryPreds:
            mln.closedWorldPreds.append(pred)
    mrf = mln.groundMRF(db, method='WCSPGroundingFactory')
    conv = WCSPConverter(mrf)
  
    resultDB = conv.getMostProbableWorldDB()
    for atom, value in resultDB.evidence.iteritems():
        if value is True:
            print atom, value
    print 
