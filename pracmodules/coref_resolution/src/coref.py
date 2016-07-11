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
from prac.core.base import PRACModule, PRACPIPE, PRAC
from prac.core.inference import PRACInferenceStep, PRACInference
from prac.core.wordnet import WordNet
from pracmln import praclog, MLNQuery, Database
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, stop, out, mergedom
from pracmln.utils.project import MLNProject
from pracmln.utils.visualization import get_cond_prob_png


log = praclog.logger(__name__)


class CorefResolution(PRACModule):
    """
    PRACmodule used to perform coreference resolution and simultaneous missing
    role inference.
    """

    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+========================+', (None, 'green', True), True)
        print colorize('| RESOLVING COREFERENCES |', (None, 'green', True), True)
        print colorize('+========================+', (None, 'green', True), True)

        # merge output dbs from senses_and_roles step, containing
        # roles inferred from multiple sentences.
        inf_step = PRACInferenceStep(pracinference, self)
        prev_step = pracinference.inference_steps[-1]
        dbs = prev_step.output_dbs
        projectpath = self.module_path
        sentences = []
        ac = None
        pngs = {}

        if len(dbs) < 2:
            # no coreferencing required - forward dbs and settings
            # from previous module
            inf_step.output_dbs = [db.copy(self.prac.mln) for db in dbs]
            inf_step.png = prev_step.png
            inf_step.applied_settings = prev_step.applied_settings
            log.info('Got single database. Nothing to do here. Passing db...')
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
                    print colorize('Loading Project {}'.format(ac), (None, 'cyan', True), True)
                    project = MLNProject.open(os.path.join(projectpath, '{}.pracmln'.format(ac)))
                except Exception:
                    inf_step.output_dbs = [db.copy(self.prac.mln) for db in dbs]
                    inf_step.png = prev_step.png
                    inf_step.applied_settings = prev_step.applied_settings
                    log.warning('Could not load project "{}". Passing dbs to next module...'.format(ac))
                    return inf_step

                # clear corefdb and unify current db with the two preceding ones
                corefdb = Database(self.prac.mln)
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
                mln = parse_mln(mlntext,
                                searchpaths=[self.module_path],
                                projectpath=projectpath,
                                logic=project.queryconf.get('logic',
                                                            'FuzzyLogic'),
                                grammar=project.queryconf.get('grammar',
                                                              'PRACGrammar'))

                # adding similarities
                wordnet_module = self.prac.getModuleByName('wn_senses')
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

                infer = MLNQuery(config=conf,
                                 db=newdatabase,
                                 mln=mln).run()

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
                pngs['CoRef - ' + str(i)] = get_cond_prob_png(project.queryconf.get('queries', ''), dbs, filename=self.name)
                inf_step.png = pngs
                inf_step.applied_settings = project.queryconf.config
        return inf_step


if __name__ == '__main__':

    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False,
                      action='store_true',
                      help="Starts PRAC inference with an interactive GUI tool.")
    (options, args) = parser.parse_args()

    sentences = args
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)

    inference = PRACInference(prac, sentences)

    if len(inference.instructions) > 0:
        parser = prac.getModuleByName('nl_parsing')
        prac.run(inference, parser)

    modules = ['ac_recognition', 'senses_and_roles', 'coref_resolution']
    for mname in modules:
        module = prac.getModuleByName(mname)
        prac.run(inference, module)
