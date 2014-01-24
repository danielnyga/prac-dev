import re
import os

class KeyIdentifier(object):
    def __init__(self, value=None):
        self.value = value
    
    def __str__(self):
        return str(self.value)
    
    def __eq__(self, other):
        return str(self) == str(other)

def toMLNCompatibleFormat(p):
    '''
        Replaces characters that are not allowed in MLN syntax, such 
        as '-' or '.' by '_'
    '''
    if type(p) == list:
        return [toMLNCompatibleFormat(x) for x in p]
    elif type(p) == KeyIdentifier:
        return toMLNCompatibleFormat(p.value)
    elif type(p) == str or type(p) == unicode:
        if type(p) == unicode:
            p = str(p)
        newValue = p.replace('-', '_')
        newValue = newValue.replace('.', '_')
        newValue = newValue.replace('\'', '')
        newValue = newValue.replace('!', '_')
        newValue = newValue.replace(',','_')
        if not newValue.isupper():
            newValue = newValue.capitalize()
        return newValue
    assert False

def load_file_return_string(file_name):
    """ 
    Load file from disk and return content as string.
    
    parameters:
        - file_name: location of the file
    returns: String of the contents of the file. If loading of the file was
             not possible return empty string.
    
    """
    res = ""
    if os.path.exists(file_name):
        try:
            with open(file_name,'r') as f:
                for l in f:
                    res += l
            f.close()
        except Exception as e:
            print("File '{filename}' does not exist or could not be " \
                  "opened.".format(filename = file_name))
    return res

def check_string_not_none_special_empty(par):
    """ 
    Test a String not to be none or contain special 
    chars, or be empty.
    
    returns:
        - True if the string is valid
    """

    if par is None:
        return False
    if par=="":
        return False
    if not re.search(r"^[\w\d'/-]+.*$", par):
        return False
    return True