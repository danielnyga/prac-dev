import json
import os
import traceback
from prac.core.base import ActionCore
from prac.core.inference import PRACInferenceStep, PRACInference
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
    distrs = {}
    success = False
    msg = ''

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
        mlntext = project.mlns.get(project.queryconf['mln'], None)
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
        success = True
        missingroles = [x for x in roles.keys() if roles[x] is None]
        for mr in missingroles:
            g = pracsession.prac.wordnet.to_dot()
            var = result.mrf.variable('has_sense({}-roleterm,_)'.format(mr))
            for atom in var.gndatoms:
                res = result.results[str(atom)]
                concept = atom.args[1]
                g.node(concept, fillcolor=get_prob_color(res))
                distrs[mr] = render_gv(g)


        success = True
        missingroles = [x for x in roles.keys() if roles[x] is None]
    except IOError:
        traceback.print_exc()
        msg = 'An MLN for the action core "{}" does not exist!'.format(ac)
    except:
        msg = 'Something went wrong. Please try again.'
    finally:
        return jsonify({'distributions': distrs, 'success': success, 'msg': msg})
