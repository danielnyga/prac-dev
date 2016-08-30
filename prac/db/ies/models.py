# 
#
# (C) 2011-2016 by Daniel Nyga (nyga@cs.uni-bremen.de)
# 
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# 'Software'), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from prac.db.ies import constants
import datetime
from prac.db.ies.constants import JSON_HOWTO_IMPORT_DATE, JSON_HOWTO_STEPS,\
    JSON_OBJECT_SYNTAX
from pracmln.mln.util import edict
from pprint import pprint
from scipy.stats import stats



def tojson(obj):
    '''Recursively generate a JSON representation of the object ``obj``.'''
    if hasattr(obj, 'tojson'): 
        return obj.tojson()
    if type(obj) in (list, tuple):
        return [tojson(e) for e in obj]
    elif isinstance(obj, dict):
        return {str(k): tojson(v) for k, v in obj.iteritems()}
    return obj
        

class Frame(object):
    '''
    Represents a (partially) instantiated action core that is stored in the MongoDB. 
    '''
    def __init__(self, prac, sidx, sentence, words, syntax, actioncore, actionroles):
        self.sidx = sidx
        self.sentence = sentence
        self.actionroles = actionroles
        self.actioncore = actioncore
        self.syntax = syntax
        self.words = words
        self.prac = prac

    
    def __str__(self):
        return '%s [%s]' % (self.actioncore, ', '.join(['%s: %s' % (k, v) for k, v in self.actionroles.items()]))
        
    
    def sim(self, f):
        '''
        Determines the frame similarity of this frame and another frame f.
        The frame similarity is calculated by taking the harmonic mean of the actionroles between the given frames.
        This value can be interpreted as the semantic similarity between the frames.
          
        :param f:     The frame this frame shall be compared to.
        :return:      The frame similarity of this frame and ``f``.
        '''
        verbsim = self.prac.wordnet.similarity(str(self.actionroles['action_verb'].type),
                                        str(f.actionroles['action_verb'].type))
        #------------------------------------------------------------------------------ 
        # This is a sanity check to revoke false inferred 
        # frames during the information extraction process.
        if verbsim  < 0.85: return 0
        #------------------------------------------------------------------------------ 
        sims = []
        for rolename, rolevalue in self.actionroles.items():
            if rolename in f.actionroles:
                sims.append(self.prac.wordnet.similarity(f.actionroles[rolename].type, rolevalue.type))
                #------------------------------------------------------------------------------ 
                #Sometimes Stanford Parser parses some objects as adjectives
                #due to the fact that nouns and adjectives cannot be compared
                #we define the the similarity between the instruction and the frame as zero
                #------------------------------------------------------------------------------ 
                if sims[-1] == 0: return 0
        return stats.hmean(sims)
    
    
    def word(self, wid):
        for w in self.words:
            if w.wid == wid: return w
            
    def object(self, oid):
        for r, o in self.actionroles.iteritems():
            if o.id == oid: return o
        
        
    
    def tojson(self):
        return {constants.JSON_FRAME_SENTENCE: self.sentence,
                constants.JSON_FRAME_ACTIONCORE: self.actioncore,
                constants.JSON_FRAME_SYNTAX: self.syntax,
                constants.JSON_FRAME_WORDS: [w.tojson() for w in self.words],
                constants.JSON_FRAME_ACTIONCORE_ROLES: self.actionroles}


    @staticmethod
    def fromjson(data):
        return Frame(sidx=data.get(constants.JSON_FRAME_SENTENCE_IDX),
                     sentence=data.get(constants.JSON_FRAME_SENTENCE),
                     syntax=data.get(constants.JSON_FRAME_SYNTAX),
                     actioncore=data.get(constants.JSON_FRAME_ACTIONCORE),
                     actionroles=data.get(constants.JSON_FRAME_ACTIONCORE_ROLES))
        
    def missingroles(self):
        return [r for r in self.prac.actioncores[self.actioncore].roles if r not in self.actionroles]
        


class Howto(Frame):
    '''
    Wrapper class representing a howto in PRAC.
    '''
    def __init__(self, prac, instr, steps, import_date=None):
        Frame.__init__(self, prac, sidx=instr.sidx, sentence=instr.sentence, syntax=instr.syntax,
                       words=instr.words, actioncore=instr.actioncore, actionroles=instr.actionroles)
        self.steps = steps
        if import_date is None:
            self.import_date = datetime.datetime.now()
        else:
            self.import_date = import_date
        
        
    def tojson(self):
        return tojson(edict({JSON_HOWTO_IMPORT_DATE: self.import_date}) +\
               edict(Frame.tojson(self)) + edict({JSON_HOWTO_STEPS: tojson(self.steps)}))
        
    
    @staticmethod
    def fromjson(data):
        return Howto(instr=Frame.fromjson(data), 
                     steps=[Frame.fromjson(s) for s in data.get(JSON_HOWTO_STEPS)],
                     import_date=data.get(JSON_HOWTO_IMPORT_DATE))


class PropertyStore(object):
    '''Store for property values of objects'''
    
    __props = ['size', 'hypernym', 'color', 'hasa', 'shape', 'dimension', 
                 'consistency', 'material']
    
    def __init__(self, prac):
        self.size = None
        self.hypernym = None
        self.color = None
        self.hasa = None
        self.shape = None
        self.dimension = None
        self.consistency = None
        self.material = None
        self.prac = prac


    def tojson(self):
        return {k: tojson(getattr(self, k)) for k in self.__props if getattr(self, k) is not None}

    
    @staticmethod
    def fromjson(data):
        s = PropertyStore()
        for k, v in data.items(): setattr(s, k, v)
        return s
    
        
class Object(object):
    '''
    Representation of a generic object that has an id and a type.
    '''
    def __init__(self, prac, id_, type_, props=None, syntax=None):
        self.type = type_
        self.id = id_
        if isinstance(props, PropertyStore):
            self.props = props
        else:
            self.props = PropertyStore(prac)
        if isinstance(props, dict):
            for k, v in props.iteritems(): setattr(self.props, k, v)
        self.syntax = syntax
        self.prac = prac
    
    
    def tojson(self):
        return tojson({constants.JSON_OBJECT_ID: self.id,
                constants.JSON_OBJECT_TYPE: self.type,
                constants.JSON_OBJECT_PROPERTIES: self.props,
                constants.JSON_OBJECT_SYNTAX: self.syntax})
        
        
    @staticmethod
    def fromjson(data):
        return Object(type_=data.get(constants.JSON_OBJECT_TYPE),
                      id_=data.get(constants.JSON_OBJECT_ID),
                      props={k: Object.fromjson(v) for k, v in data.get(constants.JSON_OBJECT_PROPERTIES, {}).iteritems()},
                      syntax=data.get(JSON_OBJECT_SYNTAX))
        
    
    def __repr__(self):
        return '<Object id=%s type=%s at 0x%x>' % (self.id, self.type, hash(self))
    
    
    def __str__(self):
        return repr(self)#'%s: %s' % (self.id, self.type)
    
        

class Word(object):
    '''
    This class represents a data structure of a word in a sentence.
    Sense objects are used to determine the meaning of a sentence.
    A sense object is always included in a frame, which contains a set of senses.
    WordNet is used to assign the senses.
    '''
    def __init__(self, prac, wid, word, widx, sense, pos, lemma, misc=None):
        self.wid = wid
        self.word = word
        self.widx = widx
        self.sense = sense
        self.pos = pos
        self.lemma = lemma
        self.misc = misc
        self.prac = prac

        
    def tojson(self):
        return tojson({constants.JSON_SENSE_WORD_ID: self.wid, 
                constants.JSON_SENSE_WORD: self.word,
                constants.JSON_SENSE_LEMMA: self.lemma,
                constants.JSON_SENSE_POS: self.pos,
                constants.JSON_SENSE_WORD_IDX: self.widx,
                constants.JSON_SENSE_SENSE: self.sense,
                constants.JSON_SENSE_MISC: self.misc})
    
    
    @staticmethod    
    def fromjson(data):
        return Word(data.get(constants.JSON_SENSE_WORD),
                    data.get(constants.JSON_SENSE_LEMMA),
                    data.get(constants.JSON_SENSE_POS),
                    data.get(constants.JSON_SENSE_SENSE),
                    data.get(constants.JSON_SENSE_WORD_IDX),
                    data.get(constants.JSON_SENSE_WORD))


if __name__ == '__main__':
    from prac.core.base import PRAC
    prac = PRAC()
    o1 = Object(prac, 'w1', 'cup.n.01', syntax=Word(prac, 'water-1', 'water', 1, 'water.n.06', 'NN', 'water'), props={'color': 'green.n.01'})
    print o1
    print repr(o1)
    pprint(o1.tojson())
