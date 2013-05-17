from django.utils import simplejson
from django.http import HttpResponse
import types
import pdb

def wr(f):
    def ret(*args, **kwargs):
        rr = f(*args, **kwargs)
        rr.update({"jsonrpc": "2.0"})
        return HttpResponse(simplejson.dumps(rr))
    return ret

def form_manager(f):
    def ret(*args, **kwargs):
        return f(*args, **kwargs)
        # assume args[0] is request
        pdb.set_trace()
        request = args[0]
        data = simplejson.loads(request.raw_post_data)
        if "jsonrpc" in data:
            request.POST = data["params"][0]
        return f(request, *args[1:], **kwargs)
    return ret

def force_positional_arguments(f):
    """decorator that checks if params are tuple containing only one dict (indicating that on client side there was call
    with positional arguments too - if so, pass values from dict as keyword arguments to decorated function"""
    def wrapped(*args, **kwargs):
        # assume args[0] is request
        request, args = args[0], args[1:]
        if len(args) == 1: # if calign code with keyword parameters from django or some other code
            return f(request, **kwargs)
        data = simplejson.loads(request.raw_post_data)
        params = data["params"]
        if isinstance(params, types.TupleType) and len(params) == 1 and isinstance(params[0], types.DictType):
            return f(request, **params[0])
        return f(request, *params, **kwargs)
    return wrapped
    

class JSONRPCService:
    def __init__(self, method_map={}):
        self.method_map = method_map
        
    def add_method(self, name, method):
        self.method_map[name] = method
        
    @wr
    def __call__(self, request, extra=None):
        #TODO: add support for jsonrpc tag
        #assert extra == None # we do not yet support GET requests, something pyjams do not use anyways.
        data = simplejson.loads(request.raw_post_data)
        id, method, params = data["id"], data["method"], data["params"]
        if method in self.method_map:
            try:
                result = self.method_map[method](request, *params)
                return {'id': id, 'result': result}
            except Exception, e:
                return {'id': id, 'error': {"code": -2, "message": str(e), "data": ""}}
        else:
            return {'id': id, 'error': {"message": "No such method", "data": "", 'code': -1}}

# FormProcessor provides a mechanism for turning Django Forms into JSONRPC
# Services.  If you have an existing Django app which makes prevalent
# use of Django Forms it will save you rewriting the app.
# use as follows.  in djangoapp/views.py :
#
# class SimpleForm(forms.Form):
#     testfield = forms.CharField(max_length=100)
#
# class SimpleForm2(forms.Form):
#     testfield = forms.CharField(max_length=20)
#
# processor = FormProcessor({'processsimpleform': SimpleForm,
#                            'processsimpleform2': SimpleForm2})
#
# this will result in a JSONRPC service being created with two
# RPC functions.  dump "processor" into urlpatterns to make it
# part of the app:
#  (r'^formsservice/$', 'djangoapp.views.processor'),

from django import forms 

def builderrors(form):
    d = {}
    for error in form.errors.keys():
        if error not in d:
            d[error] = []
        for errorval in form.errors[error]:
            d[error].append(unicode(errorval))
    return d


# contains the list of arguments in each field
field_names = {
 'CharField': ['max_length', 'min_length'],
 'IntegerField': ['max_value', 'min_value'],
 'FloatField': ['max_value', 'min_value'],
 'DecimalField': ['max_value', 'min_value', 'max_digits', 'decimal_places'],
 'DateField': ['input_formats'],
 'DateTimeField': ['input_formats'],
 'TimeField': ['input_formats'],
 'RegexField': ['max_length', 'min_length'], # sadly we can't get the expr
 'EmailField': ['max_length', 'min_length'],
 'URLField': ['max_length', 'min_length', 'verify_exists', 'user_agent'],
 'ChoiceField': ['choices'],
 'FilePathField': ['path', 'match', 'recursive', 'choices'],
 'IPAddressField': ['max_length', 'min_length'],
 }

def describe_field_errors(field):
    res = {}
    field_type = field.__class__.__name__
    msgs = {}
    for n, m in field.error_messages.items():
        msgs[n] = unicode(m)
    res['error_messages'] = msgs
    if field_type in ['ComboField', 'MultiValueField', 'SplitDateTimeField']:
        res['fields'] = map(describe_field, field.fields)
    return res

def describe_fields_errors(fields, field_names):
    res = {}
    if not field_names:
        field_names = fields.keys()
    for name in field_names:
        field = fields[name]
        res[name] = describe_field_errors(field)
    return res

from django.utils.functional import Promise
def describe_field(bfield):
    "argument name stands for BoundField, see django.forms.BoundField definition - it has interface to extract name from form's field"
    field_type = bfield.field.__class__.__name__
    res = {'type': field_type}
    for fname in field_names.get(field_type, []) + \
          ['help_text', 'label', 'initial', 'required']:
        try:
            res[fname] = getattr(bfield, fname)
        except:
            res[fname] = getattr(bfield.field, fname)
        if isinstance(res[fname], Promise): # force translation if translatable name (can't serialize some proxy obj exception)
            res[fname] = res[fname].title()
    if field_type in ['ComboField', 'MultiValueField', 'SplitDateTimeField']:#TODO: (may work but look at it)
        res['fields'] = map(describe_field, field.fields)
    return res

def describe_fields(form, field_names_to_describe = []):
    field_names_to_describe = field_names_to_describe or [bf.name for bf in form] # by default describe all fields
    return dict((bf.name, describe_field(bf)) for bf in form if bf.name in field_names_to_describe)

class FormProcessor(JSONRPCService):
    def __init__(self, forms, _formcls=None):

        if _formcls is None:
            JSONRPCService.__init__(self)
            for k in forms.keys():
                s  = FormProcessor({}, forms[k])
                self.add_method(k, s.__process)
        else:
            JSONRPCService.__init__(self, forms)
            self.formcls = _formcls

    def __process(self, request, params, command=None):

        f = self.formcls(params)

        if command is None: # just validate
            if not f.is_valid():
                return {'success':False, 'errors': builderrors(f)}
            return {'success':True}

        elif command.has_key('describe_errors'):
            field_names = command['describe_errors']
            return describe_fields_errors(f.fields, field_names)

        elif command.has_key('describe'):
            field_names = command['describe']
            ret_val = describe_fields(f, field_names)
            print 'returning from describe: ', ret_val
            print '>> params: ', params
            print '>> command: ',command
            return ret_val
        elif command.has_key('is_valid'):
            return {'success': f.is_valid() }

        elif command.has_key('save'):
            if not f.is_valid():
                return {'success':False, 'errors': builderrors(f)}
            instance = f.save() # XXX: if you want more, over-ride save.
            return {'success': True, 'instance': json_convert(instance) }

        elif command.has_key('html'):
            return {'success': True, 'html': f.as_table()}

        return "unrecognised command"




# The following is incredibly convenient for saving vast amounts of
# coding, avoiding doing silly things like this:
#     jsonresult = {'field1': djangoobject.field1,
#                   'field2': djangoobject.date.strftime('%Y.%M'),
#                    ..... }
#
# The date/time flatten function is there because JSONRPC doesn't
# support date/time objects or formats, so conversion to a string
# is the most logical choice.  pyjamas, being python, can easily
# be used to parse the string result at the other end.
#
# use as follows:
#
# jsonservice = JSONRPCService()
#
# @jsonremote(jsonservice)
# def list_some_model(request, start=0, count=10):
#     l = SomeDjangoModelClass.objects.filter()
#     res = json_convert(l[start:end])
#
# @jsonremote(jsonservice)
# def list_another_model(request, start=0, count=10):
#     l = AnotherDjangoModelClass.objects.filter()
#     res = json_convert(l[start:end])
#
# dump jsonservice into urlpatterns to make the two RPC functions,
# list_some_model and list_another_model part of the django app:
#  (r'^service1/$', 'djangoapp.views.jsonservice'),

from django.core.serializers import serialize
import datetime #remove ?
from datetime import date

def dict_datetimeflatten(item):
    d = {}
    for k, v in item.items():
        k = str(k)
        if isinstance(v, datetime.date):
            d[k] = str(v)
        elif isinstance(v, dict):
            d[k] = dict_datetimeflatten(v)
        else:
            d[k] = v
    return d

def json_convert(l, fields=None):
#   return [dict_datetimeflatten(i) for i in serialize('python', l, fields=fields)]
    res = []
    for item in serialize('python', l, fields=fields):
        res.append(dict_datetimeflatten(item))
    return res

