from prac.core import PRAC
from prac.wordnet import WordNet

class PRACFlask(object):
    '''
    The PRAC Flask app.
    '''    

    def __init__(self):
        print 'creating PRACFlask object'
        self.app = None
        self.prac = PRAC()
        self.prac.wordnet = WordNet(concepts=None)