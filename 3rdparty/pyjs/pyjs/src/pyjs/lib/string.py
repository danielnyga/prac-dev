"""The code is taken from the python2.6 tring module.

Only the non-depricated is copied.

Public module variables:

whitespace -- a string containing all characters considered whitespace
lowercase -- a string containing all characters considered lowercase letters
uppercase -- a string containing all characters considered uppercase letters
letters -- a string containing all characters considered letters
digits -- a string containing all characters considered decimal digits
hexdigits -- a string containing all characters considered hexadecimal digits
octdigits -- a string containing all characters considered octal digits
punctuation -- a string containing all characters considered punctuation
printable -- a string containing all characters considered printable

"""

# Some strings for ctype-style character classification
whitespace = ' \t\n\r\v\f'
lowercase = 'abcdefghijklmnopqrstuvwxyz'
uppercase = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
letters = lowercase + uppercase
ascii_lowercase = lowercase
ascii_uppercase = uppercase
ascii_letters = ascii_lowercase + ascii_uppercase
digits = '0123456789'
hexdigits = digits + 'abcdef' + 'ABCDEF'
octdigits = '01234567'
punctuation = """!"#$%&'()*+,-./:;<=>?@[\]^_`{|}~"""
printable = digits + letters + punctuation + whitespace

# Case conversion helpers
# Use str to convert Unicode literal in case of -U
l = map(chr, xrange(256))
_idmap = str('').join(l)
del l

# Functions which aren't available as string methods.

# Capitalize the words in a string, e.g. " aBc  dEf " -> "Abc Def".
def capwords(s, sep=None):
    """capwords(s [,sep]) -> string

    Split the argument into words using split, capitalize each
    word using capitalize, and join the capitalized words using
    join.  If the optional second argument sep is absent or None,
    runs of whitespace characters are replaced by a single space
    and leading and trailing whitespace are removed, otherwise
    sep is used to split and join the words.

    """
    if sep is None:
        sep = ' '
    return sep.join(x.capitalize() for x in s.split(sep))
    #return (sep or ' ').join(x.capitalize() for x in s.split(sep))


# Construct a translation string
_idmapL = None
def maketrans(fromstr, tostr):
    """maketrans(frm, to) -> string

    Return a translation table (a string of 256 bytes long)
    suitable for use in string.translate.  The strings frm and to
    must be of the same length.

    """
    if len(fromstr) != len(tostr):
        raise ValueError, "maketrans arguments must have same length"
    global _idmapL
    if not _idmapL:
        _idmapL = list(_idmap)
    L = _idmapL[:]
    fromstr = map(ord, fromstr)
    for i in range(len(fromstr)):
        L[fromstr[i]] = tostr[i]
    return ''.join(L)



####################################################################
import re as _re

class _multimap:
    """Helper class for combining multiple mappings.

    Used by .{safe_,}substitute() to combine the mapping and keyword
    arguments.
    """
    def __init__(self, primary, secondary):
        self._primary = primary
        self._secondary = secondary

    def __getitem__(self, key):
        try:
            return self._primary[key]
        except KeyError:
            return self._secondary[key]


# workaround for Issue #658
# drop use of __metaclass__, pyjamas support incomplete
# drop the _re.VERBOSE flag, not supported by JS
# drop the (?P<name>...) construction in the regex, not supported by JS
# escape delimiter manually, _re.escape() does not work
class Template:
    """A string class for supporting $-substitutions.
    Modify the class attributes below to change the behaviour"""
    delimiter = r'$'    #!$%^*@#~?|+ all work, &=/ do not
    idpattern = r'[_a-z][_a-z0-9]*'
    regexp = r'%(delim)s(?:(%(delim)s)|(%(id)s)|{(%(id)s)}|())'

    def __init__(self, template):
        self._definePattern()
        self.template = template

    def _definePattern(self):
        regexp = self.regexp % {
                    'delim' : '\\' + self.delimiter, # was _re.escape(delimiter)
                    'id'    : self.idpattern
                    }
        self.pattern = _re.compile(regexp, _re.IGNORECASE )

    # Search for $$, $identifier, ${identifier}, and any bare $'s

    def _invalid(self, mo):
        i = mo.start()
        j = min(i+10,len(self.template))
        raise ValueError('Invalid placeholder in string: ' + self.template[i:j])

    def substitute(self, *args, **kws):
        if len(args) > 1:
            raise TypeError('Too many positional arguments')
        if not args:
            mapping = kws
        elif kws:
            mapping = _multimap(kws, args[0])
        else:
            mapping = args[0]
        # Helper function for .sub()
        def convert(mo):
            # Check the most common path first.
            #named = mo.group('named') or mo.group('braced')
            named = mo.group(2) or mo.group(3)
            if named is not None:
                val = mapping[named]
                # We use this idiom instead of str() because the latter will
                # fail if val is a Unicode containing non-ASCII characters.
                return '%s' % (val,)
            if mo.group(1) is not None:
                return self.delimiter
            if mo.group(4) is not None:
                self._invalid(mo)
            raise ValueError('Unrecognized named group in pattern',
                             self.pattern)
        return self.pattern.sub(convert, self.template)

    def safe_substitute(self, *args, **kws):
        if len(args) > 1:
            raise TypeError('Too many positional arguments')
        if not args:
            mapping = kws
        elif kws:
            mapping = _multimap(kws, args[0])
        else:
            mapping = args[0]
        # Helper function for .sub()
        def convert(mo):
            named = mo.group(2)
            if named is not None:
                try:
                    # We use this idiom instead of str() because the latter
                    # will fail if val is a Unicode containing non-ASCII
                    return '%s' % (mapping[named],)
                except KeyError:
                    return self.delimiter + named
            braced = mo.group(3)
            if braced is not None:
                try:
                    return '%s' % (mapping[braced],)
                except KeyError:
                    return self.delimiter + '{' + braced + '}'
            if mo.group(1) is not None:
                return self.delimiter
            if mo.group(4) is not None:
                return self.delimiter
            raise ValueError('Unrecognized named group in pattern',
                             self.pattern)
        return self.pattern.sub(convert, self.template)
