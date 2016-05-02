import json
import os
import traceback
from prac.core.wordnet import known_concepts, WordNet
from prac.pracutils.pracgraphviz import render_gv
from prac.sense_distribution import get_prob_color
from pracmln import Database, MLNQuery
from pracmln.mln.base import parse_mln
from pracmln.praclog import logger
from pracmln.utils.project import MLNProject
from pracweb.gui.app import pracApp
from pracweb.gui.pages.utils import ensure_prac_session, convert, PRAC_HOME
from flask import session, jsonify, request

log = logger(__name__)


@pracApp.app.route('/prac/_change_ac', methods=['POST'])
def change_ac():
    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    actioncore = data['ac']
    ac = pracsession.prac.actioncores.get(actioncore)
    return jsonify({"roles": ac.roles})


@pracApp.app.route('/prac/_change_distr', methods=['POST'])
def change_distr():

    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    ac = data['ac']
    roles = convert(data['roles'])
    wn_module = pracsession.prac.getModuleByName('wn_senses')
    module = pracsession.prac.getModuleByName('senses_and_roles')
    success = False
    msg = ''

    # generating hash from query
    keydict = {'ac': ac}
    keydict.update(roles)
    query_hash = hash(frozenset(keydict.items()))

    # get previously hashed results, convert from unicode to string
    if os.path.isfile(os.path.join(PRAC_HOME, '.saved_distributions.json')):
        with open(os.path.join(PRAC_HOME, '.saved_distributions.json'), 'r') as infile:
            hashedresults = convert(json.load(infile))
    else:
        hashedresults = {}

    if str(query_hash) in hashedresults:
        # get distrs from saved results
        log.info('getting distrs from hashed results...')
        results = hashedresults[str(query_hash)]
    else:
        log.info('could not find saved results. calculating distrs...')
        db = Database(pracsession.prac.mln)
        db << 'action_core(action_verb-roleterm,{})'.format(ac)
        for r in roles:
            db << '{}({}-roleterm,{})'.format(r, r, ac)
            if roles[r] is None:
                continue
            db << 'has_sense({}-roleterm,{})'.format(r, roles[r])

        projectpath = os.path.join(module.module_path, '{}.pracmln'.format(ac))
        try:
            project = MLNProject.open(projectpath)
            mlntext = project.mlns.get('{}_distr.mln'.format(project.queryconf['mln'].rstrip('.mln')), None)
            mln = parse_mln(mlntext,
                            searchpaths=[module.module_path],
                            projectpath=projectpath,
                            logic=project.queryconf.get('logic', 'FuzzyLogic'),
                            grammar=project.queryconf.get('grammar', 'PRACGrammar'))

            # add inferred concepts to known_concepts to display
            # them in the graph. Ignore verbs and adjectives,
            # as they do not have hypernym relations to nouns
            # concepts = known_concepts + roles.values()
            # concepts.remove(None)
            db.domains['sense'].extend(known_concepts)
            db.domains['sense'] = list(set(db.domains['sense']))

            output_db = wn_module.add_sims(db, mln)

            infer = MLNQuery(method='EnumerationAsk',
                             mln=mln,
                             db=output_db,
                             queries='has_sense',
                             cw_preds='is_a,action_core,{}'.format(','.join(roles.keys())),
                             cw=True,
                             multicore=True,
                             verbose=True)

            result = infer.run()
            results = result.results

            # dump/overwrite result to json
            log.info('dumping results to json')
            hashedresults[query_hash] = result.results
            with open(os.path.join(PRAC_HOME,'.saved_distributions.json'), 'w') as outfile:
                outfile.write(json.dumps(convert(hashedresults)))

            success = True

        except IOError:
            traceback.print_exc()
            msg = 'An MLN for the action core "{}" does not exist!'.format(ac)
            return jsonify({'distributions': {}, 'success': success, 'msg': msg})
        except:
            msg = 'Something went wrong. Please try again.'
            return jsonify({'distributions': {}, 'success': success, 'msg': msg})

    # generate svg from results
    distrs = {}
    missingroles = [x for x in roles.keys() if roles[x] is None]
    for mr in missingroles:
        g = pracsession.prac.wordnet.to_dot()
        for res in results:
            if mr in res:
                concept = res.split(',')[1].split(')')[0]
                g.node(concept, fillcolor=get_prob_color(results[res]))
                distrs[mr] = render_gv(g)
    success = True
    return jsonify({'distributions': distrs, 'success': success, 'msg': msg})
