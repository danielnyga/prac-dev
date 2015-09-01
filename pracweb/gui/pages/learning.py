from pracmln.mln.base import parse_mln
from pracmln.mln.database import parse_db
from pracmln.mln.methods import LearningMethods
import StringIO

LEARNMETHODS = LearningMethods.names()
POSSIBLEPROPS = ['color', 'size', 'shape', 'hypernym', 'hasa']
new_usage = {
    "openWorld": "-ow",
    "maxSteps": "-maxSteps",
    "numChains": "-numChains"}

ENGINES = [('PRACMLNs', "PRACMLNs")]


def learn(data, files):
    if all(x in data for x in ['mln', 'logic', 'evidence', 'module']):
        mln = parse_mln(str(data['mln']), logic=str(data['logic']))
        trainingdbs = parse_db(mln, str(data['evidence']), ignore_unknown_preds=True)

        method = str(getattr(data, 'method', LearningMethods.DCLL))
        evidencepreds = list(getattr(data, 'evidencePreds', [])) or POSSIBLEPROPS
        params = eval("dict({})".format(str(data['parameters'])))

        trainedmln = mln.learnWeights(trainingdbs, method, evidencePreds=evidencepreds, **params)

        mlnstr = StringIO.StringIO()
        trainedmln.write(mlnstr)

        return mlnstr.getvalue()
    return None
