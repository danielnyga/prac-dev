import dill
import weakref

class _class:
    def _method(self):
        pass

class _class2:
    def __call__(self):
        pass

class _newclass(object):
    def _method(self):
        pass

class _newclass2(object):
    def __call__(self):
        pass

def _function():
    pass

o = _class()
oc = _class2()
n = _newclass()
nc = _newclass2()
f = _function
z = _class
x = _newclass

r = weakref.ref(o)
dr = weakref.ref(_class())
p = weakref.proxy(o)
dp = weakref.proxy(_class())
c = weakref.proxy(oc)
dc = weakref.proxy(_class2())

m = weakref.ref(n)
dm = weakref.ref(_newclass())
t = weakref.proxy(n)
dt = weakref.proxy(_newclass())
d = weakref.proxy(nc)
dd = weakref.proxy(_newclass2())

fr = weakref.ref(f)
fp = weakref.proxy(f)
#zr = weakref.ref(z) #XXX: weakrefs not allowed for classobj objects
#zp = weakref.proxy(z) #XXX: weakrefs not allowed for classobj objects
xr = weakref.ref(x)
xp = weakref.proxy(x)

objlist = [r,dr,m,dm,fr,xr, p,dp,t,dt, c,dc,d,dd, fp,xp]

for obj in objlist:
  res = dill.detect.errors(obj)
  if res:
    print ("%s:\n  %s" % (obj, res))
  assert not res
