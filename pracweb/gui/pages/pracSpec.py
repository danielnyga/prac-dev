from pracweb.gui.app import pracApp
from flask import redirect

@pracApp.app.route('/')
def start():
    return redirect('/prac/')

@pracApp.app.route('/log/<filename>')
def log(filename):
    return redirect('/prac/log/{}'.format(filename))