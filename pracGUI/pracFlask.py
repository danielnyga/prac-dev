from pracWEB.app import SessionStore

class PRACFlask(object):
    '''
    The PRAC Flask app.
    '''    

    def __init__(self):
        print 'creating PRACFlask object'
        self.app = None
        self.session_store = SessionStore()
