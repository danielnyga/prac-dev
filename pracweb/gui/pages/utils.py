from pracmln.utils.config import query_config_pattern, PRACMLNConfig
from pracweb.gui.app import pracApp
import os, re
import tempfile
import logging
import collections
from pracweb.gui.app import PRACSession
from prac.core.base import PRAC
from prac.core.wordnet import WordNet
from pracmln.mln.methods import InferenceMethods

FILEDIRS = {'mln': 'mln', 'pracmln': 'bin', 'db': 'db'}
LOGICS = [('FirstOrderLogic', 'FOL'), ('FuzzyLogic', 'Fuzzy')]
GRAMMAR = [('PRACGrammar', 'PRAC Grammar'), ('StandardGrammar', 'Standard Grammar')]
PRAC_HOME = os.environ['PRAC_HOME']
INFMETHODS = InferenceMethods.names()


def ensure_prac_session(session):
    log = logging.getLogger(__name__)
    prac_session = pracApp.session_store[session]
    if prac_session is None:
        session['id'] = os.urandom(24)
        prac_session = PRACSession(session)
        prac_session.prac = PRAC()
        prac_session.prac.wordnet = WordNet(concepts=None)
        # initialize the nl_parsing module so the JVM is started
        prac_session.prac.getModuleByName('nl_parsing')
        log.info('created new PRAC session %s' % str(prac_session.id.encode('base-64')))
        prac_session.tmpsessionfolder = init_file_storage()
        pracApp.session_store.put(prac_session)
    return prac_session


def convert(data):
    """
    Converts a dictionary's keys/values from unicode to string
    - data:    dictionary containing unicode keys and values
    - returns:  the converted dictionary
    """
    if isinstance(data, basestring):
        return str(data)
    elif isinstance(data, collections.Mapping):
        return dict(map(convert, data.iteritems()))
    elif isinstance(data, collections.Iterable):
        return type(data)(map(convert, data))
    else:
        return data


def update_kb_list(prac, modulename, tmpfoldername):
    kbs = []
    if modulename in prac.moduleManifestByName:
        module_path = prac.moduleManifestByName[modulename].module_path

        if not os.path.isdir(os.path.join(module_path, 'bin')): return []
        for path in os.listdir(os.path.join(module_path, 'bin')):
            if os.path.isdir(path): continue
            if path.endswith('.pracmln'):
                kbs.append(path[0:path.rfind('.pracmln')])

    if os.path.isdir(os.path.join(tmpfoldername, 'bin')):
        for path in os.listdir(os.path.join(tmpfoldername, 'bin')):
            if path.endswith('.pracmln'):
                kbs.append(path[0:path.rfind('.pracmln')])

    return [(kb, kb) for kb in kbs]


def update_mln_list(prac, modulename, tmpfoldername):
    mlns = []
    if modulename in prac.moduleManifestByName:
        module_path = prac.moduleManifestByName[modulename].module_path

        for path in os.listdir(os.path.join(module_path, 'mln')):
            if os.path.isdir(path): continue
            if path.endswith('.mln'):
                mlns.append(path[0:path.rfind('.mln')])

    if os.path.isdir(os.path.join(tmpfoldername, 'mln')):
        for path in os.listdir(os.path.join(tmpfoldername, 'mln')):
            if path.endswith('.mln'):
                mlns.append(path[0:path.rfind('.mln')])

    return [('{}.mln'.format(mln), '{}.mln'.format(mln)) for mln in mlns]


def update_evidence_list(prac, modulename, tmpfoldername):
    evidence = []
    if modulename in prac.moduleManifestByName:
        module_path = prac.moduleManifestByName[modulename].module_path

        if not os.path.isdir(os.path.join(module_path, 'db')): return []
        for path in os.listdir(os.path.join(module_path, 'db')):
            if os.path.isdir(path): continue
            if path.endswith('.db'):
                evidence.append(path[0:path.rfind('.db')])

    if os.path.isdir(os.path.join(tmpfoldername, 'db')):
        for path in os.listdir(os.path.join(tmpfoldername, 'db')):
            if path.endswith('.db'):
                evidence.append(path[0:path.rfind('.db')])

    return [('{}.db'.format(ev), '{}.db'.format(ev)) for ev in evidence]


# returns content of given file, replaces includes by content of the included file
def get_file_content(fdir, fname):
    c = ''
    if os.path.isfile(os.path.join(fdir, fname)):
        with open(os.path.join(fdir, fname), "r") as f:
            c = f.readlines()

    content = ''
    for l in c:
        if '#include' in l:
            # includefile = re.sub('#include (.*[.].*$)', '\g<1>', l).strip()
            includefile = re.sub('#include ([\w,\s-]+\.[A-Za-z])', '\g<1>', l).strip()
            content += get_file_content(fdir, includefile)
        else:
            content += l
    return content


def save_kb(kb, tmpfoldername, name=None):
    if name is None and not hasattr(kb, 'name'):
        raise Exception('No name specified.')
    config = PRACMLNConfig(os.path.join(tmpfoldername, 'bin', query_config_pattern % name if name is not None else kb.name))
    config.dump()


def add_wn_similarities(db, concepts, wn):
    known_concepts = [wn.synset(c) for c in db.mln.domains['concept']]
    evidence_concepts = [wn.synset(c) for c in concepts]

    for kc in known_concepts:
        for ec in evidence_concepts:
            sim = wn.wup_similarity(kc, ec)
            db.addGroundAtom('is_a({},{})'.format(kc.name, ec.name), sim)


def init_file_storage():
    if not os.path.exists(os.path.join(pracApp.app.config['UPLOAD_FOLDER'])):
        os.mkdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER']))
    dirname = tempfile.mkdtemp(prefix='pracweb', dir=pracApp.app.config['UPLOAD_FOLDER'])

    if not os.path.exists(os.path.join(pracApp.app.config['LOG_FOLDER'])):
        os.mkdir(os.path.join(pracApp.app.config['LOG_FOLDER']))

    return dirname

