"""Define names for all type symbols known in the standard interpreter.

Types that are part of optional modules (e.g. array) are not listed.
"""
import sys

# Iterators in Python aren't a matter of type but of protocol.  A large
# and changing number of builtin types implement *some* flavor of
# iterator.  Don't check the type!  Use hasattr to check for both
# "__iter__" and "next" attributes instead.

NoneType = type(None)
TypeType = type
ObjectType = object

IntType = int
LongType = long
FloatType = float
BooleanType = bool
try:
    ComplexType = complex
except NameError:
    pass

StringType = str

# StringTypes is already outdated.  Instead of writing "type(x) in
# types.StringTypes", you should use "isinstance(x, basestring)".  But
# we keep around for compatibility with Python 2.2.
try:
    UnicodeType = unicode
    StringTypes = (StringType, UnicodeType)
except NameError:
    StringTypes = (StringType,)

BufferType = buffer

TupleType = tuple
ListType = list
DictType = DictionaryType = dict

def _f(): pass
FunctionType = type(_f)
LambdaType = type(lambda: None)         # Same as FunctionType

# XXX: funcs dont have func_code
CodeType = __builtins__.CodeType

def _g():
    yield 1
GeneratorType = type(_g())

class _C:
    def _m(self): pass
ClassType = type(_C)
UnboundMethodType = type(_C._m)         # Same as MethodType
_x = _C()
InstanceType = type(_x)
MethodType = type(_x._m)

BuiltinFunctionType = type(len)
BuiltinMethodType = type([].append)     # Same as BuiltinFunctionType

ModuleType = type(sys)
FileType = file
XRangeType = xrange

# XXX: These are represented via native js objects
TracebackType = __builtins__.TracebackType
FrameType = __builtins__.FrameType

SliceType = slice
EllipsisType = type(Ellipsis)

# XXX: dictproxy is not checked in yet
#DictProxyType = type(TypeType.__dict__)

NotImplementedType = type(NotImplemented)

# For Jython, the following two types are identical
GetSetDescriptorType = __builtins__.CodeType
MemberDescriptorType = __builtins__.CodeType

del sys, _f, _g, _C, _x                           # Not for export
