from webrob.pracinit import pracApp

def register_routes():

    from webrob.app import app
    pracApp.app = app
    from webrob.pages import pracSpec

    from webrob.pages import views
    from webrob.pages import utils
