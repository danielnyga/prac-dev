from flask import Flask

class PRACFlask(object):
    '''
    The PRAC Flask app.
    '''

    def __init__(self):
        print 'creating PRACFlask object'
        self.app = None
        self.session_store = SessionStore()

class PRACSession():
    
    def __init__(self, http_session):
        self.id = http_session['id']
        self.http_session = http_session

class SessionStore():
    
    def __init__(self):
        self.sessions = {}

    def put(self, prac_session):
        self.sessions[prac_session.id] = prac_session
        
    def __getitem__(self, s):
        if 'id' not in s: return None
        return self.sessions.get(s['id'])
    
    def remove(self, s):
        del self.sessions[s['id']]
        
    def __str__(self):
        return str(self.sessions)
    
app = Flask(__name__)
pracApp = PRACFlask()