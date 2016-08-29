# PROBABILISTIC ROBOT ACTION CORES
#
# (C) 2016 by Mareike Picklum (mareikep@cs.uni-bremen.de)
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
import traceback

from prac.core.base import PRACModule, PRACPIPE, PRACDatabase
from prac.core.inference import PRACInferenceStep
from prac.pracutils.utils import prac_heading
from pracmln import praclog
from pracmln.mln import NoConstraintsError, MLNParsingError
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, mergedom
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


logger = praclog.logger(__name__, praclog.INFO)


class CorefResolution(PRACModule):
    '''
    PRACModule used to perform coreference resolution and simultaneous missing
    role inference.
    '''

    @PRACPIPE
    def __call__(self, pracinference, **params):

        # ======================================================================
        # Initialization
        # ======================================================================

        logger.debug('inference on {}'.format(self.name))

        if self.prac.verbose > 0:
            print prac_heading('Resolving Coreferences')

        inf_step = PRACInferenceStep(pracinference, self)
        prev_step = pracinference.inference_steps[-1]
        dbs = prev_step.output_dbs
        projectpath = self.module_path
        sentences = []
        ac = None
        pngs = {}

        # ======================================================================
        # Preprocessing
        # ======================================================================

        # merge output dbs from senses_and_roles step, containing
        # roles inferred from multiple sentences.
        if len(dbs) < 2:
            # no coreferencing required - forward dbs and settings
            # from previous module
            inf_step.output_dbs = [db.copy(self.prac.mln) for db in dbs]
            inf_step.png = prev_step.png
            inf_step.applied_settings = prev_step.applied_settings
            logger.debug('Got single database. Nothing to do here. Passing db...')
            return inf_step

        # retrieve all words from the dbs to calculate distances.
        # Do not use pracinference.instructions as they are not
        # annotated by the Stanford parser.
        for s in range(0, len(dbs)):
            sen = [x['?w'] for x in list(dbs[s].query('has_pos(?w,?pos)'))]
            sentences.append(sen)

        for i, db in enumerate(dbs):
            if i == 0:
                # no coreference resolution required for first database
                inf_step.output_dbs.append(dbs[i])
            else:
                # query action core to load corresponding project
                for q in dbs[i].query('action_core(?w,?ac)'):
                    ac = q['?ac']

                try:
                    logger.debug('Loading Project: {}'.format(colorize(ac, (None, 'cyan', True), True)))
                    project = MLNProject.open(os.path.join(projectpath, '{}.pracmln'.format(ac)))

                    # clear corefdb and unify current db with the two preceding ones
                    corefdb = PRACDatabase(self.prac)
                    for s in range(max(0, i - 2), i+1):
                        corefdb = corefdb.union(dbs[s], self.prac.mln)

                    # remove all senses from the databases' domain, that are not
                    # assigned to any word.
                    for q in corefdb.query('!(EXIST ?w (has_sense(?w,?sense)))'):
                        corefdb.rmval('sense', q['?sense'])

                    # preprocessing: adding distance information for each
                    # word in the instructions
                    s = sentences[max(0, i - 2):i+1]
                    snts = list(enumerate(s))
                    idx = len(snts) - 1  # idx of current sentence
                    for s in snts[:-1]:
                        idx2 = s[0]
                        for w in s[1]:
                            corefdb << 'distance({},DIST{})'.format(w, idx - idx2)

                    mlntext = project.mlns.get(project.queryconf['mln'], None)
                    mln = parse_mln(mlntext, searchpaths=[self.module_path],
                                    projectpath=projectpath,
                                    logic=project.queryconf.get('logic', 'FuzzyLogic'),
                                    grammar=project.queryconf.get('grammar', 'PRACGrammar'))
                except MLNParsingError:
                    logger.warning('Could not use MLN in project {} for coreference resolution'.format(colorize(ac, (None, 'cyan', True), True)))
                    inf_step.output_dbs = [db.copy(self.prac.mln) for db in dbs]
                    inf_step.png = prev_step.png
                    inf_step.applied_settings = prev_step.applied_settings
                    return inf_step
                except IOError:
                    inf_step.output_dbs = [db.copy(self.prac.mln) for db in dbs]
                    inf_step.png = prev_step.png
                    inf_step.applied_settings = prev_step.applied_settings
                    logger.warning('A model for "{}" does not exist. Passing dbs to next module...'.format(ac))
                except Exception:
                    inf_step.output_dbs = [db.copy(self.prac.mln) for db in dbs]
                    inf_step.png = prev_step.png
                    inf_step.applied_settings = prev_step.applied_settings
                    logger.warning('Something went wrong. Passing dbs to next module...')
                    return inf_step

                # adding similarities
                wordnet_module = self.prac.module('wn_senses')
                newdatabase = wordnet_module.add_sims(corefdb, mln)

                # update queries depending on missing roles
                acroles = filter(lambda role: role != 'action_verb', self.prac.actioncores[ac].roles)
                missingroles = [ar for ar in acroles if len(list(newdatabase.query('{}(?w,Adding)'.format(ar)))) == 0]
                conf = project.queryconf
                conf.update({'queries': ','.join(missingroles)})
                print colorize('querying for {}'.format(conf['queries']), (None, 'green', True), True)

                # asserting impossible role-ac combinations, leaving previously
                # inferred roles untouched
                fulldom = mergedom(mln.domains, newdatabase.domains)
                ac_domains = [dom for dom in fulldom if '_ac' in dom]
                acs = list(set([v for a in ac_domains for v in fulldom[a]]))
                acs = filter(lambda ac_: ac_ != ac, acs)

                for ac1 in acs:
                    for r in missingroles:
                        for w in newdatabase.domains['word']:
                            # words with no sense are asserted false
                            if list(corefdb.query('!(EXIST ?sense (has_sense({},?sense)))'.format(w))):
                                newdatabase << '!{}({},{})'.format(r, w, ac)
                            # leave previously inferred information roles
                            # untouched
                            if list(newdatabase.query('{}({},{})'.format(r, w, ac1))):
                                continue
                            else:
                                newdatabase << '!{}({},{})'.format(r, w, ac1)

                try:
                    # ==========================================================
                    # Inference
                    # ==========================================================

                    infer = self.mlnquery(config=conf,
                                          verbose=self.prac.verbose > 2,
                                          db=newdatabase, mln=mln)

                    if self.prac.verbose == 2:
                        print
                        print prac_heading('INFERENCE RESULTS')
                        infer.write()

                    # ==========================================================
                    # Postprocessing
                    # ==========================================================

                    # merge initial db with results
                    for res in infer.results.keys():
                        if infer.results[res] != 1.0:
                            continue
                        db << '{}'.format(res)
                        w = res.split('(')[1].split(',')[0]
                        for q in newdatabase.query('has_sense({0},?s) ^ has_pos({0},?pos)'.format(w)):
                            db << 'has_sense({},{})'.format(w, q['?s'])
                            db << 'is_a({0},{0})'.format(q['?s'])
                            db << 'has_pos({},{})'.format(w, q['?pos'])

                    newdb = wordnet_module.add_sims(db, mln)
                    inf_step.output_dbs.append(newdb)
                except NoConstraintsError:
                    logger.debug('No coreferences found. Passing db...')
                    inf_step.output_dbs.append(db)
                except Exception:
                    logger.error('Something went wrong')
                    traceback.print_exc()

                pngs['Coref - ' + str(i)] = get_cond_prob_png(project.queryconf.get('queries', ''), dbs, filename=self.name)
                inf_step.png = pngs
                inf_step.applied_settings = project.queryconf.config
        return inf_step
