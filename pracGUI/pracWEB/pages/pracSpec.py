from pracWEB.pracinit import pracApp
from flask import redirect

@pracApp.app.route('/')
def start():
    return redirect('/prac/')

@pracApp.app.route('/log')
def log():
    return 'PRAC Log'