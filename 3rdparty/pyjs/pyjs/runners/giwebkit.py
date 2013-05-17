# Copyright (C) 2012 C Anthony Risinger <anthony@xtfx.me>
#
# LICENSE: Apache 2.0 <http://www.apache.org/licenses/LICENSE-2.0.txt>

import os
import sys
import logging
logging.basicConfig(level=logging.INFO)
logging.getLogger(__name__).setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)

import re
from urllib import urlopen
from urlparse import urljoin

import gi
gi.require_version('Gtk', '3.0')
gi.require_version('WebKit', '3.0')
from gi.repository import GObject
from gi.repository import Gtk
from gi.repository import Soup
from gi.repository import WebKit

import types
import signal
import operator
from traceback import print_exc
from pprint import pformat
from uuid import uuid4


sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)
sys.stderr = os.fdopen(sys.stderr.fileno(), 'w', 0)


#TODO: impl collections.MutableMapping
class URI(object):

    KEYS = [f.get_name() for f in Soup.URI.__info__.get_fields()]

    get_keys = staticmethod(operator.attrgetter(*KEYS))

    @staticmethod
    def items(uri):
        return zip(URI.KEYS, URI.get_keys(uri))


class GIXMLHttpRequestEventTarget(object):

    onloadstart = None
    onprogress = None
    onabort = None
    onerror = None
    onload = None
    ontimeout = None
    onloadend = None


class GIXMLHttpRequestUpload(GIXMLHttpRequestEventTarget):

    pass


class GIXMLHttpRequest(GIXMLHttpRequestEventTarget):
    '''XMLHttpRequest Level 2 http://www.w3.org/TR/XMLHttpRequest/'''

    UNSENT = 0
    OPENED = 1
    HEADERS_RECEIVED = 2
    LOADING = 3
    DONE = 4

    ResponseType = set(['', 'arraybuffer', 'blob',
                        'document', 'json','text'])

    _app = None
    _msg = None
    _meta = {
        '_session': WebKit.get_default_session(),
        'source-origin': 'about:blank',
        'upload': None,
        'status': 0,
        'status-text': '',
        'ready-state': UNSENT,
        'response-type': '',
        'response-text': '',
        'anonymous-flag': None,
        'send-flag': None,
        'error-flag': None,
        'synchronous-flag': None,
        'request-method': 'GET',
        'request-url': 'about:blank',
        'request-username': None,
        'request-password': None,
        'author-request-headers': None,
        'request-entity-body': None,
        'upload-complete-flag': None,
        'upload-events-flag': None,
        'credentials-flag': None,
        'force-preflight-flag': None,
        'cross-origin-request-status': None,
        'timeout': 0,
        }

    #TODO: properties ...
    onreadystatechange = None
    response = None #ro, any
    responseXML = None #DOMDocument (or XMLDOC-thing)

    @property
    def timeout(self):
        return self._meta['timeout']

    @timeout.setter
    def timeout(self, timeout):
        #TODO: more to this?
        self._meta['timeout'] = timeout

    @property
    def withCredentials(self):
        return self._meta['credentials-flag']

    @withCredentials.setter
    def withCredentials(self, withCredentials):
        #TODO: more to this?
        self._meta['credentials-flag'] = withCredentials

    @property
    def responseText(self):
        return self._meta['response-text']

    @property
    def upload(self):
        return self._meta['upload']

    @property
    def status(self):
        return self._meta['status']

    @property
    def statusText(self):
        return self._meta['status-text']

    @property
    def readyState(self):
        return self._meta['ready-state']

    @property
    def responseType(self):
        return self._meta['response-type']

    @responseType.setter
    def responseType(self, responseType):
        if responseType not in self.ResponseType:
            raise TypeError('InvalidAccessError')
        if self.readyState in (self.LOADING, self.DONE):
            raise TypeError('InvalidStateError')
        #TODO: there are 1-2 more check, but really ... ?
        self._meta['response-type'] = responseType

    def __init__(self, app):
        self._app = app

    def open(self, method, url, async=True, user=None, password=None):
        import pygwt
        self.abort()
        async = async and True or False
        user = user or None
        password = password or None
        view = self._app._view
        uri = Soup.URI.new(pygwt.getModuleBaseURL()).new_with_base(url)
        uri.set_user(user)
        uri.set_password(password)
        msg = Soup.Message.new_from_uri(method, uri)
        uri = msg.get_uri()
        self._msg = msg
        self._meta = GIXMLHttpRequest._meta.copy()
        self._meta.update({
            'source-origin': uri.copy_host().to_string(0).rstrip('/'),
            'upload': GIXMLHttpRequestUpload(),
            'ready-state': self.OPENED,
            'synchronous-flag': not async and True or None,
            'request-method': method.upper(),
            'request-url': uri.to_string(0),
            'request-username': user,
            'request-password': password,
            'author-request-headers': {
                'user-agent': view.get_settings().get_user_agent(),
                },
            })

    def setRequestHeader(self, header, value):
        if self.readyState != 1:
            raise TypeError('InvalidStateError')
        self._meta['author-request-headers'][header.lower()] = value

    def send(self, data=None):
        msg = self._msg
        meta = self._meta
        hdrs = meta['author-request-headers']
        if self.readyState != self.OPENED or meta['send-flag']:
            raise TypeError('InvalidStateError')
        if data and meta['request-method'] not in ('HEAD', 'GET'):
            meta['request-entity-body'] = data
            msg.request_body.append(data)
            #TODO: content-Type ...
        #TODO: storage mutex? prob doesn't apply ...
        if not meta['synchronous-flag']:
            for key in GIXMLHttpRequestEventTarget.__dict__:
                if callable(getattr(self.upload, key)):
                    meta['upload-events-flag'] = True
                    break
        meta['error-flag'] = None
        if not meta['request-entity-body']:
            meta['upload-complete-flag'] = True
        if not meta['synchronous-flag']:
            meta['send-flag'] = True
            #TODO: fire event: readystatechange
            #TODO: fire progress event: loadstart
            pass
            if not meta['upload-complete-flag']:
                #TODO: fire progress event: loadstart (upload)
                pass
        if 'accept' not in meta['author-request-headers']:
            hdrs['accept'] = '*/*'
        for hdr in hdrs.iteritems():
            msg.request_headers.replace(*hdr)
        if not meta['anonymous-flag']:
            msg.request_headers.replace('origin', meta['source-origin'])
        #TODO: app_frame == Referer?
        if meta['request-url'].startswith(meta['source-origin']):
            #TODO: should be async
            meta['status'] = int(meta['_session'].send_message(msg))
            meta['response-text'] = str(msg.response_body.data)
            meta['ready-state'] = self.DONE
            if callable(self.onreadystatechange):
                #TODO: wrong signature
                self.onreadystatechange(self, None, None)
        else:
            #TODO: CORS? :-(
            pass

    def abort(self):
        msg = self._msg
        meta = self._meta
        if (self.readyState in (self.UNSENT, self.OPENED) and
                meta['send-flag']) or self.readyState == self.DONE:
            meta['ready-state'] = self.DONE
            #TODO: fire event: readystatechange
            #TODO: fire progress event: abort
            #TODO: fire progress event: loadend
            pass
            if not meta['upload-complete-flag']:
                #TODO: fire progress event: abort (upload)
                #TODO: fire progress event: loadend (upload)
                pass
        meta['ready-state'] = self.UNSENT

    def getResponseHeader(self, header):
        raise NotImplemented('XMLHttpRequest.getResponseHeader')
        return ''

    def getAllResponseHeaders(self):
        raise NotImplemented('XMLHttpRequest.getAllResponseHeaders')
        return '...\r\n...\r\n'

    def overrideMimeType(self, mime):
        raise NotImplemented('XMLHttpRequest.overrideMimeType')


class GITimer(object):

    UUID = uuid4().hex

    key = None

    def __init__(self, key):
        self.key = key

    def __call__(self, wnd, cb, ms):
        doc = wnd.document
        ctx = doc.ctx

        buf = doc.createTextNode(self.key)
        sig = doc.createEvent('MouseEvent')
        sig.initMouseEvent(
            self.UUID, 0, 0, wnd, ms, 0,
            0, 0, 0, 0, 0, 0, 0, 0, buf
            )

        wnd.dispatch_event(sig)
        ctx.addEventListener(buf, self.UUID[::-1], cb)
        return int(buf.data)

    @classmethod
    def bind(cls, key):
        owner, attr = key
        return types.MethodType(cls(attr), None, owner)


class GIProxy(object):

    key = None
    getter = None
    setter = None

    def __init__(self, key, impl='property'):
        self.key = key
        self.getter = operator.methodcaller('get_%s' % impl, key)
        self.setter = operator.attrgetter('set_%s' % impl)

    def __get__(self, inst, type_gi):
        return self.getter(inst)

    def __set__(self, inst, attr):
        self.setter(inst)(self.key, attr)

    def __delete__(self, inst):
        pass


class GIWindowLocation(object):

    _ctx = None

    def __init__(self, ctx):
        #TODO: use SoupURI for this instead
        Soup.URI.new()
        def update(doc, pspec=None):
            a.set_href(doc.get_document_uri())
            logger.debug('location:%s', a.get_href())
        doc = app._doc
        a = doc.createElement('a')
        doc.connect('notify::document-uri', update)
        update(doc)
        object.__setattr__(self, '_app', app)
        object.__setattr__(self, '_a', a)

    def __getattr__(self, key):
        return getattr(self._a, key)

    def __setattr__(self, key, attr):
        #TODO: needs to interact with view.load_uri()
        setattr(self._a, key, attr)

    def assign(self):
        #TODO
        self._app._wnd.get_history()

    def reload(self):
        self._app._view.reload()

    def replace(self):
        #TODO
        self._app._wnd.get_history()

    @classmethod
    def bind(cls, key):
        owner, attr = key
        return types.MethodType(cls(key), None, owner)

class GIWindowOpen(object):

    def __init__(self, ctx):
        pass

    def __call__(self, inst, uri, name="_blank", specs=""):
        if '://' not in uri:
            import pygwt
            uri = Soup.URI.new(pygwt.getModuleBaseURL()).new_with_base(uri).to_string(0)
        rc = RunnerContext()
        rc._destroy_cb = lambda *args: logger.debug('destroying sub window...')
        rc.setup(uri)

    @classmethod
    def bind(cls, key):
        owner, attr = key
        return types.MethodType(cls(key), None, owner)


class GIResolver(object):

    NONE = object()
    UPPER = re.compile('([A-Z])')

    _custom = {
        (WebKit.DOMDOMWindow, 'location'): GIWindowLocation,
        (WebKit.DOMDOMWindow, 'open'): GIWindowOpen,
        (WebKit.DOMDOMWindow, 'setTimeout'): GITimer,
        (WebKit.DOMDOMWindow, 'setInterval'): GITimer,
        #TODO: this is actually a bug in pyjs ... UIEvents
        #      do not have these attributes.
        (WebKit.DOMUIEvent, 'shiftKey'): False,
        (WebKit.DOMUIEvent, 'ctrlKey'): False,
        (WebKit.DOMUIEvent, 'altKey'): False,
        }

    _type_gi = None

    def __init__(self, type_gi):
        method = types.MethodType(self, None, type_gi)
        type.__setattr__(type_gi, '__getattr__', method)
        type.__setattr__(type_gi, '__setattr__', method)
        self._type_gi = type_gi

    def __call__(self, inst, key, attr=NONE):
        if attr is self.NONE:
            return self.getattr(inst, key)
        self.setattr(inst, key, attr)

    def getattr(self, inst, key):
        for impl in (self.getattr_gi, self.getattr_w3):
            attr = impl(inst, key)
            if attr is not self.NONE:
                logger.debug('%s:%s.%s', impl.__name__,
                    inst.__class__.__name__, key)
                return attr
        raise AttributeError('%r object has no attribute %r' % (
                                    inst.__class__.__name__, key))

    def getattr_gi(self, inst, key):
        try:
            if inst.get_data(key) is None:
                return self.NONE
        except TypeError:
            return self.NONE
        type.__setattr__(inst.__class__, key, GIProxy(key, 'data'))
        return getattr(inst, key)

    def getattr_w3(self, inst, key_w3):
        key_gi = self._key_gi(key_w3)
        for base in inst.__class__.__mro__:
            key = (base, key_w3)
            if key in self._custom:
                try:
                    attr = self._custom[key].bind(key)
                except AttributeError:
                    attr = self._custom[key]
            elif hasattr(inst.props, key_gi):
                attr = GIProxy(key_gi)
            elif key_gi in base.__dict__:
                attr = base.__dict__[key_gi]
            else:
                continue
            type.__setattr__(base, key_w3, attr)
            return getattr(inst, key_w3)
        return self.NONE

    def setattr(self, inst, key, attr):
        # hasattr() *specifically* chosen because it calls getattr()
        # internally, possibly setting a proxy object; if True, super()
        # will then properly setattr() against the proxy or instance.
        if hasattr(inst, key):
            super(self._type_gi, inst).__setattr__(key, attr)
        else:
            inst.set_data(key, attr)
            logger.debug('setattr(inst, %r, attr):\n%s', key,
                pformat([('inst', inst), ('attr', attr)]))

    def _key_gi(self, key):
        return self.UPPER.sub(r'_\1', key).lower()


class Callback(object):

    def __init__(self, sender, cb, boolparam):
        self.sender = sender
        self.cb = cb
        self.boolparam = boolparam

    def _callback(self, sender, event, data):
        try:
            return self.cb(self.sender, event, self.boolparam)
        except:
            print_exc()
            return None


class ApplicationFrame(object):

    #TODO: split RunnerContext (multi-frame support)
    pass


class RunnerContext(object):

    platform = 'webkit'
    uri = 'about:blank'
    #TODO: rename, accidentally removed?
    appdir = None
    width = 800
    height = 600
    # TODO: change WebKit patch to hold reference
    listeners = None

    def __init__(self):
        self.listeners = dict()

    def run(self):
        logger.debug('mainloop:entering...')
        Gtk.main()
        logger.debug('mainloop:exiting...')

    def setup(self, uri=uri, **kwds):
        for k, v in kwds.iteritems():
            if hasattr(self, k):
                setattr(self, k, v)
        if '://' not in uri:
            uri = 'file://%s' % os.path.abspath(uri)

        uri = self.uri = Soup.URI.new(uri)
        logger.info('uri:\n%s', pformat(URI.items(uri)))

        view = self._view = WebKit.WebView()
        toplevel = self._toplevel = Gtk.Window()
        scroller = self._scroller = Gtk.ScrolledWindow()
        toplevel.set_default_size(self.width, self.height)
        toplevel.add(scroller)
        scroller.add(view)

        char_q, mod_ctrl = Gtk.accelerator_parse('<Ctrl>q')
        accel_destroy = Gtk.AccelGroup.new()
        accel_destroy.connect(char_q, mod_ctrl, 0, self._destroy_cb)

        #TODO: file:/// with # or ? causes error
        view.load_uri(self.uri.to_string(0))

        view.connect('onload-event', self._frame_loaded_cb)
        view.connect('title-changed', self._title_changed_cb)
        view.connect('icon-loaded', self._icon_loaded_cb)
        view.connect('populate-popup', self._populate_popup_cb)
        view.connect('console-message', self._console_message_cb)
        #view.connect('resource-content-length-received',
        #             self._resource_recv_cb, None)
        #view.connect('resource-request-starting',
        #             self._resource_init_cb, None)

        settings = view.get_property('settings')
        settings.set_property('auto-resize-window', True)
        settings.set_property('enable-file-access-from-file-uris', True)
        settings.set_property('enable-accelerated-compositing', True)
        settings.set_property('enable-webgl', True)

        # GLib.PRIORITY_LOW == 300
        GObject.timeout_add(1000, self._idle_loop_cb, priority=300)
        signal.signal(signal.SIGINT, self._destroy_cb)
        toplevel.connect('destroy', self._destroy_cb)
        toplevel.add_accel_group(accel_destroy)

        # display and run mainloop (returns after frame load)
        toplevel.show_all()
        Gtk.main()

    def getUri(self):
        return self.uri.to_string(0)

    def _idle_loop_cb(self):
        # mostly here to enable Ctrl^C/SIGINT without resorting to:
        #     signal.signal(signal.SIGINT, signal.SIG_DFL)
        # ... but could be useful in future; active timeout required else
        # SIGINT is not be processed until the next Gdk event
        return True

    def _console_message_cb(self, view, message, lineno, source):
        logger.debug('JAVASCRIPT:%s:%s', lineno, message)
        return True

    def _resource_init_cb(self, view, frame, webres, req, res, data):
        m = req.get_message()
        content_type = m.request_headers.get_content_type()[0]
        if content_type is not None:
            if content_type.startswith('application/json'):
                print 'JSONRPC!'

    def _resource_recv_cb(self, view, frame, res, length, data):
        #m = res.get_network_request().get_message()
        #print m.request_headers.get_content_type()
        print res.props.mime_type, res.props.uri

    def _frame_loaded_cb(self, view, frame):
        #TODO: multiple apps should be simple to implement
        if frame is not view.get_main_frame():
            logger.debug('sub-frame: %s', frame)
            return

        self._doc = self._view.get_dom_document()
        self._wnd = self._doc.get_default_view()
        self._doc.ctx = self

        # GITimer: ready the listener
        view.execute_script(r'''
            (function(wnd, doc, uujs, uugi, undefined){
                wnd.addEventListener(uujs, function(e){
                    var buf = e.relatedTarget;
                    var evt = doc.createEvent('Event');
                    evt.initEvent(uugi, 0, 0);
                    buf.data = wnd[buf.data](function(){
                        buf.dispatchEvent(evt);
                    }, e.detail);
                });
            })(window, document, %r, %r);
            ''' % (GITimer.UUID, GITimer.UUID[::-1]))

        #TODO: redundant? incompat with poly-frame/reload!
        import __pyjamas__
        __pyjamas__.set_gtk_module(Gtk)
        __pyjamas__.set_main_frame(self)

        #TODO: made this work ... and skip bootstrap.js
        #for m in __pyjamas__.pygwt_processMetas():
        #    minst = module_load(m)
        #    minst.onModuleLoad()

        # return control to setup()
        Gtk.main_quit()

    def _icon_loaded_cb(self, view, icon_uri):
        current = view.get_property('uri')
        dom = view.get_dom_document()
        icon = (Gtk.STOCK_DIALOG_QUESTION, None, 0)
        found = set()
        found.add(icon_uri)
        found.add(urljoin(current, '/favicon.ico'))
        scanner = {
            'href': dom.querySelectorAll(
                        'head link[rel~=icon][href],'
                        'head link[rel|=apple-touch-icon][href]'
                        ),
            'content': dom.querySelectorAll(
                        'head meta[itemprop=image][content]'
                        ),
            }
        for attr in scanner.keys():
            for i in xrange(scanner[attr].length):
                uri = getattr(scanner[attr].item(i), attr)
                if len(uri) == 0:
                    continue
                found.add(urljoin(current, uri))
        for uri in found:
            fp = urlopen(uri)
            if fp.code != 200:
                continue
            i = fp.info()
            if i.maintype == 'image' and 'content-length' in i:
                try:
                    ldr = Gtk.gdk.PixbufLoader()
                    ldr.write(fp.read(int(i['content-length'])))
                    ldr.close()
                except:
                    continue
                pb = ldr.get_pixbuf()
                pbpx = pb.get_height() * pb.get_width()
                if pbpx > icon[2]:
                    icon = (uri, pb, pbpx)
        if icon[1] is None:
            self._toplevel.set_icon_name(icon[0])
        else:
            self._toplevel.set_icon(icon[1])
        logger.debug('icon:%s', icon[0])

    def mash_attrib(self, name, joiner='-'):
        return name

    def alert(self, msg):
        self._wnd.alert(msg)

    def _populate_popup_cb(self, view, menu):
        menu.append(Gtk.SeparatorMenuItem.new())
        go = lambda signal, uri: view.load_uri(uri)
        entries = [
            ('About WebKit', 'http://live.gnome.org/WebKitGtk'),
            ('About pyjs.org', 'http://pyjs.org/About.html'),
            ]
        for label, uri in entries:
            entry = Gtk.MenuItem(label=label)
            entry.connect('activate', go, uri)
            menu.append(entry)
        menu.show_all()

    def getDomWindow(self):
        return self._wnd

    def getDomDocument(self):
        return self._doc

    def getXmlHttpRequest(self):
        return GIXMLHttpRequest(self)

    def addWindowEventListener(self, event_name, cb):
        cb = Callback(self, cb, True)
        listener = WebKit.dom_create_event_listener(cb._callback, None)
        self._wnd.add_event_listener(event_name, listener, False)
        self.listeners[listener] = self._wnd

    def addXMLHttpRequestEventListener(self, element, event_name, cb):
        cb = Callback(element, cb, True)
        setattr(element, "on%s" % event_name, cb._callback)

    def addEventListener(self, element, event_name, cb):
        cb = Callback(element, cb, False)
        listener = WebKit.dom_create_event_listener(cb._callback, None)
        element.add_event_listener(event_name, listener, False)
        self.listeners[listener] = element

    def _destroy_cb(self, *args):
        logger.debug('destroy:draining events...')
        Gtk.main_quit()

    def _title_changed_cb(self, view, frame, title):
        self._toplevel.set_title(title)

    window = property(getDomWindow)
    document = property(getDomDocument)

    _alert = alert
    _addEventListener = addEventListener
    _addWindowEventListener = addWindowEventListener
    _addXMLHttpRequestEventListener = addXMLHttpRequestEventListener


resolver = GIResolver(WebKit.DOMObject)
context = RunnerContext()
setup = context.setup
run = context.run
