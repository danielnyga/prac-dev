#
# This is taken from the python module gettext
# Some things won't work (ugettext/ungettext/install)
# These might work in a subclass of NullTranslations

class NullTranslations:
    def __init__(self, fp=None):
        self._info = {}
        self._charset = None
        self._output_charset = None
        self._fallback = None
        if fp is not None:
            self._parse(fp)

    def _parse(self, fp):
        pass

    def add_fallback(self, fallback):
        if self._fallback:
            self._fallback.add_fallback(fallback)
        else:
            self._fallback = fallback

    def gettext(self, message):
        if self._fallback:
            return self._fallback.gettext(message)
        return message

    def lgettext(self, message):
        if self._fallback:
            return self._fallback.lgettext(message)
        return message

    def ngettext(self, msgid1, msgid2, n):
        if self._fallback:
            return self._fallback.ngettext(msgid1, msgid2, n)
        if n == 1:
            return msgid1
        else:
            return msgid2

    def lngettext(self, msgid1, msgid2, n):
        if self._fallback:
            return self._fallback.lngettext(msgid1, msgid2, n)
        if n == 1:
            return msgid1
        else:
            return msgid2

    def ugettext(self, message):
        if self._fallback:
            return self._fallback.ugettext(message)
        return unicode(message)

    def ungettext(self, msgid1, msgid2, n):
        if self._fallback:
            return self._fallback.ungettext(msgid1, msgid2, n)
        if n == 1:
            return unicode(msgid1)
        else:
            return unicode(msgid2)

    def info(self):
        return self._info

    def charset(self):
        return self._charset

    def output_charset(self):
        return self._output_charset

    def set_output_charset(self, charset):
        self._output_charset = charset

    def install(self, unicode=False, names=None):
        import __builtin__
        __builtin__.__dict__['_'] = unicode and self.ugettext or self.gettext
        if hasattr(names, "__contains__"):
            if "gettext" in names:
                __builtin__.__dict__['gettext'] = __builtin__.__dict__['_']
            if "ngettext" in names:
                __builtin__.__dict__['ngettext'] = (unicode and self.ungettext
                                                             or self.ngettext)
            if "lgettext" in names:
                __builtin__.__dict__['lgettext'] = self.lgettext
            if "lngettext" in names:
                __builtin__.__dict__['lngettext'] = self.lngettext

