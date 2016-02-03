from pracmln.praclog import logger
import threading


log = logger(__name__)


class RequestBuffer(object):

    def __init__(self):
        self._condition = threading.Condition()
        self._content = {'status': False, 'message': ''}
        self._dirty = False


    def waitformsg(self, timeout=None):
        with self.condition:
            if self._dirty:
                return
            self.condition.wait(timeout=timeout)


    def setmsg(self, cnt):
        with self.condition:
            self.content.update(cnt)
            self._dirty = True
            self.condition.notifyAll()


    @property
    def condition(self):
        return self._condition


    @property
    def content(self):
        self._dirty = False
        return self._content
