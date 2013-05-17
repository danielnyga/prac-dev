# PyJS does not support weak references,
# so this module provides stubs with usual references

typecls = __builtins__.TypeClass

class ReferenceType(typecls):
    pass
class CallableProxyType(typecls):
    pass
class ProxyType(typecls):
    pass

ProxyTypes = (ProxyType, CallableProxyType)

WeakValueDictionary = dict
WeakKeyDictionary = dict

