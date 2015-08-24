from pracweb.gui.app import pracApp
import os, re
import logging
import json
import collections
from pracweb.gui.app import PRACSession
from prac.core.base import PRAC
from prac.core.wordnet import WordNet
from pracmln.mln.methods import InferenceMethods

FILEDIRS = {'mln':'mln', 'pracmln':'bin', 'db':'db'}
LOGICS = [('FirstOrderLogic','FOL'),('FuzzyLogic','Fuzzy')]
GRAMMAR = [('PRACGrammar','PRAC Grammar'), ('StandardGrammar','Standard Grammar')]
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
        pracApp.session_store.put(prac_session)
        initFileStorage()
    return prac_session


def convert(data):
    '''
    Converts a dictionary's keys/values from unicode to string
    - data:    dictionary containing unicode keys and values
    - returns:  the converted dictionary
    '''
    if isinstance(data, basestring):
        return str(data)
    elif isinstance(data, collections.Mapping):
        return dict(map(convert, data.iteritems()))
    elif isinstance(data, collections.Iterable):
        return type(data)(map(convert, data))
    else:
        return data


def updateKBList(prac, modulename):
    kbs = []
    if modulename in prac.moduleManifestByName:
        module_path = prac.moduleManifestByName[modulename].module_path

        if not os.path.isdir(os.path.join(module_path, 'bin')): return []
        for path in os.listdir(os.path.join(module_path, 'bin')):
            if os.path.isdir(path): continue
            if path.endswith('.pracmln'):
                kbs.append(path[0:path.rfind('.pracmln')])

    if os.path.isdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'bin')):
        for path in os.listdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'bin')):
            if path.endswith('.pracmln'):
                kbs.append(path[0:path.rfind('.pracmln')])

    return [(kb,kb) for kb in kbs]


def updateMLNList(prac, modulename):
    mlns = []
    if modulename in prac.moduleManifestByName:
        module_path = prac.moduleManifestByName[modulename].module_path

        for path in os.listdir(os.path.join(module_path, 'mln')):
            if os.path.isdir(path): continue
            if path.endswith('.mln'):
                mlns.append(path[0:path.rfind('.mln')])

    if os.path.isdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'mln')):
        for path in os.listdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'mln')):
            if path.endswith('.mln'):
                mlns.append(path[0:path.rfind('.mln')])

    return [('{}.mln'.format(mln),'{}.mln'.format(mln)) for mln in mlns]


def updateEvidenceList(prac, modulename):
    evidence = []
    if modulename in prac.moduleManifestByName:
        module_path = prac.moduleManifestByName[modulename].module_path

        if not os.path.isdir(os.path.join(module_path, 'db')): return []
        for path in os.listdir(os.path.join(module_path, 'db')):
            if os.path.isdir(path): continue
            if path.endswith('.db'):
                evidence.append(path[0:path.rfind('.db')])

    if os.path.isdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'db')):
        for path in os.listdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'db')):
            if path.endswith('.db'):
                evidence.append(path[0:path.rfind('.db')])

    return [('{}.db'.format(ev),'{}.db'.format(ev)) for ev in evidence]


# returns content of given file, replaces includes by content of the included file
def getFileContent(fDir, fName):
    c = ''
    if os.path.isfile(os.path.join(fDir, fName)):
        with open (os.path.join(fDir, fName), "r") as f:
            c = f.readlines()

    content = ''
    for l in c:
        if '#include' in l:
            # includefile = re.sub('#include (.*[.].*$)', '\g<1>', l).strip()
            includefile = re.sub('#include ([\w,\s-]+\.[A-Za-z])', '\g<1>', l).strip()
            content += getFileContent(fDir, includefile)
        else:
            content += l
    return content


def save_kb(kb, name=None):
    '''
    Pickles the state of the given kb in the uploadfolder.
    - kb:    instance of a PRACKnowledgeBase
    - name:  name of the PRACKnowledgeBase
    '''
    import pickle
    if name is None and not hasattr(kb, 'name'):
        raise Exception('No name specified.')
    binaryFileName = '{}.pracmln'.format(name if name is not None else kb.name)
    binPath = os.path.join(pracApp.app.config['UPLOAD_FOLDER'], 'bin')
    if not os.path.exists(binPath):
        os.mkdir(binPath)
    f = open(os.path.join(binPath, binaryFileName), 'w+')
    pickle.dump(kb, f)
    f.close()


def add_wn_similarities(db, concepts, wn):
    known_concepts = [wn.synset(c) for c in db.mln.domains['concept']]
    evidence_concepts = [wn.synset(c) for c in concepts]

    for kc in known_concepts:
        for ec in evidence_concepts:
            sim = wn.wup_similarity(kc, ec)
            db.addGroundAtom('is_a({},{})'.format(kc.name, ec.name), sim)


def initFileStorage():
    if not os.path.exists(os.path.join(pracApp.app.config['UPLOAD_FOLDER'])):
       os.mkdir(os.path.join(pracApp.app.config['UPLOAD_FOLDER']))

    if not os.path.exists(os.path.join(pracApp.app.config['LOG_FOLDER'])):
       os.mkdir(os.path.join(pracApp.app.config['LOG_FOLDER']))

