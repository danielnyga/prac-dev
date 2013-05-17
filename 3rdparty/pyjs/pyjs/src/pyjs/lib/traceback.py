import sys

def format_exception(etype, value, tb, limit=None):
    return sys._get_traceback_list(value, tb, limit=limit)

def print_exc():
    print sys._get_traceback_list(value, tb, None)
