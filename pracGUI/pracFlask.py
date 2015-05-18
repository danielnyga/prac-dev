from prac.core import PRAC
from prac.wordnet import WordNet
from pracWEB.app import SessionStore

class PRACFlask(object):
    '''
    The PRAC Flask app.
    '''    

    def __init__(self):
        print 'creating PRACFlask object'
        self.app = None
        self.session_store = SessionStore()
#         self.prac = PRAC()
#         self.prac.wordnet = WordNet(concepts=None)