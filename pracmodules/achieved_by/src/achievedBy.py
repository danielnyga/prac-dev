# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2012-2015 by Daniel Nyga (nyga@cs.tum.edu)
# (C) 2015 by Sebastian Koralewski (seba@informatik.uni-bremen.de)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
import os
from prac.core.base import PRACModule, PRACPIPE
from prac.core.inference import PRACInferenceStep
from prac.pracutils.utils import prac_heading
from pracmln import Database, MLNQuery
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, out
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


log = logger(__name__)

class AchievedBy(PRACModule):
    """

    """

    def initialize(self):
        pass

    def shutdown(self):
        pass

    def extendDBWithAchievedByEvidence(self, db, querymln):
        actioncore = ""
        # It will be assumed that there is only one true action_core
        # predicate per database
        for q in db.query("action_core(?w,?ac)"):
            actioncore = q["?ac"]
        acdomain = querymln.domains.get("actioncore")
        acdomain.extend(db.domains.get("actioncore"))
        acdomain = set(acdomain)
        db_ = Database(self.prac.mln)

        for ac1 in acdomain:
            for ac2 in acdomain:
                if ac1 == actioncore:
                    continue
                db_ << ("achieved_by({},{})".format(ac1, ac2), 0)

        for atom, truth in sorted(db.evidence.iteritems()):
            db_ << (atom, truth)

        return db_

    @PRACPIPE
    def __call__(self, pracinference, **params):

        print prac_heading('Refining Action Cores')

        inf_step = PRACInferenceStep(pracinference, self)
        dbs = pracinference.inference_steps[-1].output_dbs

        for olddb in dbs:
            #To handle multiple acs in one task, we have to check if the single dbs contain achieved_bys which representing already plans
            skip_db = False
            mod = self.prac.getModuleByName('roles_transformation')
            plans = mod.getPlanList()
            for q in olddb.query('achieved_by(?w,?ac)'):
                actioncore = q['?ac']
                
                if actioncore in plans:
                    skip_db = True 
            
            for q in olddb.query('action_core(?w,?ac)'):
                actioncore = q['?ac']
                
                if actioncore in plans:
                    skip_db = True
             
            if skip_db:
                inf_step.output_dbs.append(olddb)
                continue

            pngs = {}
            for q in olddb.query('action_core(?w,?ac)'):
                actioncore = q['?ac']
                
                
                # This list is used to avoid an infinite loop during the
                # achieved by inference.
                # To avoid this infinite loop, the list contains the pracmlns
                # which were inferenced during the process.
                # Every pracmln should be used only once during the process
                # because the evidence for the inference will always remain
                # the same.
                # So if the pracmln hadnt inferenced a plan in the first time,
                # it will never do it.

                # Need to remove possible achieved_by predicates from
                # previous achieved_by inferences
                db_ = Database(self.prac.mln)
                for atom, truth in sorted(olddb.evidence.iteritems()):
                    if 'achieved_by' in atom:
                        continue
                    db_ << (atom,truth)

                if params.get('project', None) is None:
                    log.info('Loading Project: %s.pracmln' % colorize(actioncore, (None, 'cyan', True), True))
                    projectpath = os.path.join(self.module_path,
                                               '{}.pracmln'.format(actioncore))
                    if os.path.exists(projectpath):
                        project = MLNProject.open(projectpath)
                    else:
                        inf_step.output_dbs.append(olddb)
                        out(actioncore)
                        log.error(actioncore + ".pracmln does not exist.")
                        return inf_step
                else:
                    log.info(colorize('Loading Project from params',
                                      (None, 'cyan', True), True))
                    projectpath = os.path.join(params.get('projectpath', None) or self.module_path,
                                               params.get('project').name)
                    project = params.get('project')

                mlntext = project.mlns.get(project.queryconf['mln'], None)
                mln = parse_mln(mlntext, searchpaths=[self.module_path],
                                projectpath=projectpath,
                                logic=project.queryconf.get('logic',
                                                            'FirstOrderLogic'),
                                grammar=project.queryconf.get('grammar',
                                                              'PRACGrammar'))
                known_concepts = mln.domains.get('concept', [])
                wordnet_module = self.prac.getModuleByName('wn_senses')
                
                #Merge domains of db and given mln to avoid errors due to role inference and the resulting missing fuzzy perdicates
                known_concepts = list(set(known_concepts).union(set(db_.domains.get('concept', []))))
                db = wordnet_module.get_senses_and_similarities(db_,
                                                                known_concepts)


                unified_db = db.union(db_)
                dbnew = wordnet_module.add_sims(unified_db, unified_db)

                # Inference achieved_by predicate
                db_ = self.extendDBWithAchievedByEvidence(dbnew, mln)

                infer = MLNQuery(config=project.queryconf,
                                 db=db_, mln=mln).run()
                result_db = infer.resultdb

                # unified_db = result_db.union(kb.query_mln, db_)
                # only add inferred achieved_by atoms, leave out
                # 0-evidence atoms
                for qa in result_db.query('achieved_by(?ac1,?ac2)'):
                    unified_db << 'achieved_by({},{})'.format(qa['?ac1'],
                                                              qa['?ac2'])
                    pngs[qa['?ac2']] = get_cond_prob_png(project.queryconf.get('queries', ''), dbs, filename=self.name)

                inf_step.output_dbs.append(unified_db)
                inf_step.png = pngs

                inf_step.applied_settings = project.queryconf.config
        return inf_step
