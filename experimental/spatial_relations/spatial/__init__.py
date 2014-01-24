import sys
import os
from actioncore import PRAC
from MLN.util import strFormula
from actioncore import PRAC, PRACReasoner, PRACPIPE
from pracmln.PRACDatabase import PRACDatabase
from wcsp.converter import WCSPConverter
action_cores_spat_probs = os.path.join('models', 'spatial_relation.yaml')
class SPATLearner(object):
    
    def __init__(self, actioncore):
        self.actioncore = actioncore
    def run(self):
        action_core = self.actioncore 
        dbs = action_core.mln_spatial.loadPRACDatabases('models/'+action_core.name+'/spatial_relations/db')
        action_core.learnedMLN_spatial = action_core.mln_spatial.duplicate()
        for f_templ in action_core.formula_spatial_templates:
            print f_templ
            action_core.learnedMLN_spatial.addFormula(f_templ, hard=f_templ.isHard)
        action_core.learnedMLN_spatial.learnWeights(dbs)
        
class SPATInference(PRACReasoner):
    def __init__(self, actioncore):
        self.prac = PRAC()
        self.actioncore = self.prac.action_cores[actioncore]
    def run(self, name):
        action_core=self.actioncore
        finall_stage=False
        counter=0
        while finall_stage == False:
            my_db=[]
            solut_atoms=[]
            existing_preds=[]
            atoms_new=[]
            mln = action_core.learnedMLN_spatial.duplicate()
            if counter == 0 :
                db = PRACDatabase(mln, "models/"+action_core.name+"/spatial_relations/db_inference/"+name+".db")
            else:
                db = PRACDatabase(mln, "models/"+action_core.name+"/spatial_relations/db_inference/"+name+"_Stage"+str(counter)+".db")
            for atom, val in db.evidence.iteritems():
                existing_preds.append(atom)
            if action_core.name == 'Filling' :
                queryPreds = ['on_next', 'in_next', 'over_next', 'tilted_next','begin_state','end_state']
            if action_core.name == 'Flipping' :
                queryPreds = ['on_next', 'in_next', 'over_next', 'tilted_next','end_state','contact_next' ]
            for pred in mln.predicates:
                if not pred in queryPreds:
                    mln.closedWorldPreds.append(pred)
            mrf = mln.groundMRF(db, simplify=True, method='WCSPGroundingFactory')
            conv = WCSPConverter(mrf)
            resultDB = PRACDatabase(mln, db=conv.getMostProbableWorldDB())
            for atom, value in resultDB.evidence.iteritems():
                if value == True : atoms_new.append(atom)
            for x in atoms_new:
                if x not in existing_preds:
                    solut_atoms.append(x)     
            my_db.append(open("models/"+action_core.name+"/spatial_relations/db_inference/"+name+"_Stage"+str(counter)+".db", 'a'))
            for st in solut_atoms:
                if  st.find("end_state") == 0 : finall_stage=True
            if finall_stage == True: 
                print >> my_db[0], "end_state(YES)"
                my_db[0].close()
                break
            my_db.append(open("models/"+action_core.name+"/spatial_relations/db_inference/"+name+"_Stage"+str(counter+1)+".db", 'a'))
            if counter == 0 :
                for i in range(0,len(atoms_new)):
                    print >> my_db[0], atoms_new[i]
            else:
                for i in range(0,len(solut_atoms)):
                    print >> my_db[0], solut_atoms[i]
            for x in existing_preds:
                if  x.find("has_role") == 0 : print >> my_db[1], x
            for st in solut_atoms:
                if st != 'begin_state(YES)':
                    temp_st=st.partition('_next')
                    print >> my_db[1], temp_st[0]+temp_st[2]
            my_db[0].close()
            my_db[1].close()
            counter=counter + 1
            if counter > 7 : finall_stage=True