from mln.database import readDBFromString
from mln.mln import readMLNFromString
from mln.methods import LearningMethods
from pracWEB.pages.fileupload import upload
from pracWEB.pages.utils import updateMLNList, updateEvidenceList, GRAMMAR, LOGICS
import os, sys
import StringIO

LEARNMETHODS = [(LearningMethods.byName(method),method) for method in LearningMethods.name2value]
POSSIBLEPROPS = ['color', 'size', 'shape', 'hypernym', 'hasa']
new_usage = {
    "openWorld": "-ow",
    "maxSteps": "-maxSteps",
    "numChains": "-numChains"}

ENGINES = [ ('PRACMLNs', "PRACMLNs"),
            ({"path": r"/usr/wiss/jain/work/code/alchemy-2009-07-07/bin", "usage": new_usage},"Alchemy - July 2009 (AMD64)"),
            ({"path": r"/usr/wiss/jain/work/code/alchemy-2008-06-30/bin/amd64", "usage": new_usage},"Alchemy - June 2008 (AMD64)"),
            ({"path": os.getenv("ALCHEMY_HOME"), "usage": new_usage},"Alchemy - August 2010 (AMD64)"),
            ({"path": r"c:\users\Domini~1\Research\code\alchemy-2010-08-23\bin", "usage": new_usage},"Alchemy (Win32 desktop)"),
            ({"path": r"c:\research\code\alchemy\bin", "usage": new_usage},"Alchemy (Win32 laptop)")]



def learn(data, files):
    if all(x in data for x in ['mln','logic','evidence','module']):
        mln = readMLNFromString(str(data['mln']),str( data['logic']))
        trainingDBs = readDBFromString(mln, str(data['evidence']), ignoreUnknownPredicates=True)
        method = str(getattr(data, 'method', LearningMethods.DCLL))
        evidencePreds = list(getattr(data, 'evidencePreds', [])) or POSSIBLEPROPS
        params = eval("dict({})".format(str(data['parameters'])))

        trainedMLN = mln.learnWeights(trainingDBs, method, evidencePreds=evidencePreds, **params)

        mlnStr = StringIO.StringIO()
        trainedMLN.write(mlnStr)

        return mlnStr.getvalue()
    return None