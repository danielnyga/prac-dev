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
import itertools
from prac.core.base import PRACModule, PRACPIPE, PRAC
from prac.core.inference import PRACInferenceStep, PRACInference
from prac.core.wordnet import WordNet
from pracmln import praclog, MLNQuery, Database
from pracmln.mln.base import parse_mln
from pracmln.mln.util import colorize, stop
from pracmln.utils.project import MLNProject, PRACMLNConfig
from pracmln.utils.visualization import get_cond_prob_png


log = praclog.logger(__name__)


class CorefResolution(PRACModule):
    """
    PRACmodule used to perform coreference resolution and simultaneous missing
    role inference.
    """


    @PRACPIPE
    def __call__(self, pracinference, **params):
        print colorize('+==========================================+',
                       (None, 'green', True), True)
        print colorize('| PRAC INFERENCE: RESOLVING COREFERENCES    ',
                       (None, 'green', True), True)
        print colorize('+==========================================+',
                       (None, 'green', True), True)

        # merge output dbs from senses and roles step, containing
        # roles inferred from multiple sentences.
        dbs = pracinference.inference_steps[-1].output_dbs
        corefdb = Database(self.prac.mln)
        ac = None
        sentences = []

        # unify current db with the 3 preceding ones
        for i, db in enumerate(dbs):
            if i == 0:
                continue
            else:
                for q in dbs[i].query('action_core(?w,?ac)'):
                    ac = q['?ac']
                print colorize('Loading Project {}'.format(ac),
                               (None, 'cyan', True),
                               True)
                log.info(colorize('Loading Project {}'.format(ac),
                                  (None, 'cyan', True),
                                  True))
                projectpath = self.module_path
                project = MLNProject.open(
                    os.path.join(projectpath, '{}.pracmln'.format(ac)))

                # retrieve all words from the dbs to calculate distances
                # do not use pracinference.instructions as they are not
                # modified by the Stanford parser.
                for s in range(max(0, i - 2), i + 1):
                    sen = [x['?w'] for x in
                           list(dbs[s].query('has_pos(?w,?pos)'))]
                    sentences.append(sen)
                    corefdb = corefdb.union(dbs[s], self.prac.mln)

                # remove all senses from the databases' domain, that are not
                # assigned to any word.
                for q in corefdb.query('!(EXIST ?w (has_sense(?w,?sense)))'):
                    corefdb.rmval('sense', q['?sense'])

                # preprocessing: adding distance information for each
                # word-word pair in the instructions
                words = list(
                    enumerate([list(enumerate(s)) for s in sentences]))
                sentencecombinations = list(
                    itertools.combinations_with_replacement(
                        list(enumerate(words)), 2))

                for sc in sentencecombinations:
                    for w1 in sc[0][1][1]:
                        for w2 in sc[1][1][1]:
                            if w1[1] == w2[1]: continue
                            d = sc[0][0] - sc[1][0]
                            if d < 0:
                                dist = abs(d)
                                corefdb << 'distance({},{},DIST{})'.format(
                                    w2[1], w1[1], dist)
                            elif d == 0:
                                corefdb << 'distance({},{},DIST{})'.format(
                                    w1[1], w2[1], 0)
                                corefdb << 'distance({},{},DIST{})'.format(
                                    w2[1], w1[1], 0)
                            else:
                                dist = abs(d)
                                corefdb << 'distance({},{},DIST{})'.format(
                                    w1[1], w2[1], dist)

                mlntext = project.mlns.get(project.queryconf['mln'], None)
                mln = parse_mln(mlntext,
                                searchpaths=[self.module_path],
                                projectpath=projectpath,
                                logic=project.queryconf.get('logic',
                                                            'FuzzyLogic'),
                                grammar=project.queryconf.get('grammar',
                                                              'PRACGrammar'))
                inf_step = PRACInferenceStep(pracinference, self)

                # adding similarities
                wordnet_module = self.prac.getModuleByName('wn_senses')
                newdatabase = wordnet_module.add_sims(corefdb, mln)

                # update queries depending on missing roles
                acroles = filter(lambda r: r != 'action_verb', self.prac.actioncores[ac].roles)
                missingroles = [x for x in acroles if len(
                    list(newdatabase.query('{}(?w,Adding)'.format(x)))) == 0]
                conf = project.queryconf
                conf.update({'queries': ','.join(missingroles)})
                print colorize('querying for {}'.format(conf['queries']),
                               (None, 'green', True), True)

                # asserting impossible role-ac combinations, leaving previously
                # inferred roles untouched
                acs = list(set(newdatabase.domains['actioncore'] + mln.domains[
                    'actioncore']))
                acs.remove(ac)
                for w in newdatabase.domains['word']:
                    for ac1 in acs:
                        for r in missingroles:
                            if list(newdatabase.query('{}({},{})'.format(r, w, ac1))):
                                continue
                            else:
                                newdatabase << '!{}({},{})'.format(r, w, ac1)

                infer = MLNQuery(config=conf,
                                 db=newdatabase,
                                 mln=mln).run()
                result_db = infer.resultdb

                result_db.write()

                inf_step.output_dbs.append(result_db)

                png, ratio = get_cond_prob_png(
                    project.queryconf.get('queries', ''),
                    dbs, filename=self.name)
                inf_step.png = (png, ratio)
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
