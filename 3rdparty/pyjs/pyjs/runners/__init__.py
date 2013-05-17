import os
import sys
import logging
logging.basicConfig(level=logging.INFO)
logging.getLogger(__name__).setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)

from pprint import pformat
from os import path
import ConfigParser


class RunnerManager(object):

    _platforms = {
        None: 'hulahop',
        'win32': 'mshtml',
        }

    _conf = {
        'home': path.join(path.expanduser('~'), '.pyjd'),
        'runner': _platforms.get(sys.platform, _platforms[None]),
        'is_desktop': True,
        'native_dnd': True,
        }

    _listeners = None
    _modname = None
    _runner = None

    def __init__(self):
        self._conf = self._conf.copy()
        self._listeners = []

    def set_conf(self, rc=path.join(_conf['home'], 'pyjdrc')):
        conf = self._conf
        cf = ConfigParser.ConfigParser()
        cf.read(rc)
        if cf.has_section('gui'):
            conf.update(dict(cf.items('gui')))
            conf.setdefault('engine', conf['runner'])
            conf['runner'] = conf['engine']
        logger.info('conf:\n%s', pformat(conf.items()))

    def set_runner(self, runner=None):
        if runner is None:
            runner = self._conf.get('runner')
            if runner is None:
                raise ValueError(runner)
        if runner == 'hulahop':
            impl = 'pyjs.runners.hula'
        else:
            impl = 'pyjs.runners.%s' % runner
        __import__(impl)
        import pyjd
        pyjd.engine = runner
        import importers
        importers._test_revamp()
        self._runner = sys.modules[impl]


    def add_setup_listener(self, listener):
        self._listeners.append(listener)

    def setup(self, *args, **kwds):
        self._runner.setup(*args, **kwds)
        for listener in self._listeners:
            listener()

    def run(self, *args, **kwds):
        self._runner.run(*args, **kwds)
