from pracWEB.pracinit import pracApp
from flask import request, jsonify, session, send_from_directory, redirect, \
    render_template
from prac.inference import PRACInference
import os, re
import logging
import json
import collections
import StringIO
from geoip import geolite2
from pracWEB.app import PRACSession
from prac.core import PRAC
from prac.wordnet import WordNet
from mln.methods import InferenceMethods

FILEDIRS = {'mln':'mln', 'pracmln':'bin', 'db':'db'}
LOGICS = [('FirstOrderLogic','FOL'),('FuzzyLogic','Fuzzy')]
GRAMMAR = [('PRACGrammar','PRAC Grammar'), ('StandardGrammar','Standard Grammar')]
PRAC_HOME = os.environ['PRAC_HOME']
INFMETHODS = [(InferenceMethods.byName(method),method) for method in InferenceMethods.name2value]

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

@pracApp.app.route('/prac/log')
def praclog():
    return praclog_('null')

@pracApp.app.route('/prac/log/<filename>')
def praclog_(filename):
    if os.path.isfile(os.path.join(pracApp.app.config['LOG_FOLDER'], filename)):
        return send_from_directory(pracApp.app.config['LOG_FOLDER'], filename)
    elif os.path.isfile(os.path.join(pracApp.app.config['LOG_FOLDER'],
                                     '{}.json'.format(filename))):
        return send_from_directory(pracApp.app.config['LOG_FOLDER'], '{}.json'.format(filename))
    else:
        return render_template('userstats.html', **locals())

@pracApp.app.route('/prac/_user_stats', methods=['POST'])
def user_stats():
    ulog = logging.getLogger('userstats')

    data = convert(json.loads(request.get_data()))
    print 'user_stats', data
    print 'ip from request', request.remote_addr
    ip = data['ip'] if data['ip'] is not None else request.remote_addr
    stats = {}

    logstr = ("Wrote log entry:\n"
                    "IP:\t\t\t\t{ip}\n"
                    "Country:\t\t{country}\n"
                    "Continent:\t\t{continent}\n"
                    "Subdivisions:\t{subdivisions}\n"
                    "Timezone:\t\t{timezone}\n"
                    "Location:\t\t{location}\n"
                    "Access Date:\t{date}\n"
                    "Access Time:\t{time}")

    try:
        geolite = geolite2.lookup(ip)
        stats.update(geolite.to_dict())
        stats['subdivisions'] = ', '.join(stats['subdivisions']) # prettify for log
    except AttributeError:
        print 'using reduced logging string'
        logstr = ("Wrote log entry:\n"
                    "IP:\t\t\t\t{ip}\n"
                    "Access Date:\t{date}\n"
                    "Access Time:\t{time}")
    except ValueError:
        print 'Not a valid ip address:', ip
    except KeyError:
        logstr = ("Wrote log entry:\n"
                    "IP:\t\t\t\t{ip}\n"
                    "Country:\t\t{country}\n"
                    "Continent:\t\t{continent}\n"
                    "Timezone:\t\t{timezone}\n"
                    "Location:\t\t{location}\n"
                    "Access Date:\t{date}\n"
                    "Access Time:\t{time}")
    finally:
        stats.update({'ip': ip, 'date':data['date'], 'time':data['time']})
        ulog.info(json.dumps(stats))
        print logstr.format(**stats)
        return ''

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

@pracApp.app.route('/prac/_get_modules', methods=['GET'])
def get_modules():
    pracsession = ensure_prac_session(session)
    print 'infmethods', INFMETHODS
    return jsonify( modules=[module for module in pracsession.prac.moduleManifestByName], methods=INFMETHODS)

@pracApp.app.route('/prac/_load_flow_chart', methods=['GET'])
def _load_flow_chart():
    filename = os.path.join(os.path.join(PRAC_HOME, 'etc'), 'prac-flowchart.svg')
    with open(filename, 'r') as svgFile:
        content = svgFile.readlines()
    return ''.join(content)

@pracApp.app.route('/prac/update_module', methods=['POST'])
def updateModule():
    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    module = data.get('module')
    kbList = [kb[0] for kb in updateKBList(pracsession.prac, module)]
    mlnList = [mln[0] for mln in updateMLNList(pracsession.prac, module)] 
    evidenceList = [ev[0] for ev in updateEvidenceList(pracsession.prac, module)] 

    return jsonify({'value': module,'kblist': kbList, 'mlnlist': mlnList, 'evidencelist': evidenceList})


@pracApp.app.route('/prac/updateUploadedFiles', methods=['GET'])
def updateUploadedFiles():
    mlnList = [mln[0] for mln in updateMLNList(None)] 
    evidenceList = [ev[0] for ev in updateEvidenceList(None)] 
    ret_data = {'mlnlist': mlnList, 'evidencelist': evidenceList}

    return jsonify(ret_data)


@pracApp.app.route('/prac/update_text', methods=['POST'])
def updateText():
    pracsession = ensure_prac_session(session)
    data = json.loads(request.get_data())
    fileName = data['fName']
    moduleName = data['module']

    # look for file in upload folder
    for root, subFolders, files in os.walk(pracApp.app.config['UPLOAD_FOLDER']):
        if fileName in files:
            filePathModule = os.path.join(root, fileName)
            text = getFileContent(root, fileName)
            return jsonify({'text': text})
    
    # look for file in module path
    if moduleName in pracsession.prac.moduleManifestByName:
        module_path = pracsession.prac.moduleManifestByName[moduleName].module_path
        for root, subFolders, files in os.walk(module_path):
            if fileName in files:
                text = getFileContent(root, fileName)
                return jsonify({'text': text})

    return jsonify({'text': ''})


@pracApp.app.route('/prac/updateKB', methods=['GET'])
def updateKB():
    pracsession = ensure_prac_session(session)
    if not request.args.get('module') in pracsession.prac.moduleManifestByName or not request.args.get('kb'): 
        return jsonify({})
    module = pracsession.prac.getModuleByName(request.args.get('module'))
    kb = module.load_pracmt(request.args.get('kb'))
    res = kb.query_params
    res['mln'] = kb.query_mln_str
    return jsonify(res)


@pracApp.app.route('/prac/parse/', methods=['GET'])
def parse():
    print 'Running NL Parsing'
    txt = request.args.get('text')
    print txt
    infer = PRACInference(pracsession.prac, [txt])
    parser = pracsession.prac.getModuleByName('nl_parsing')
    pracsession.prac.run(infer, parser)

    res = ''
    for db in infer.inference_steps[-1].output_dbs:
        dbStr = StringIO.StringIO()
        db.write(dbStr)
        res += dbStr.getvalue()
    print 'res ', res

    return jsonify({'text': res})


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

