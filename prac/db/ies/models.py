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
from prac.db.ies.constants import JSON_HOWTO_IMPORT_DATE, JSON_HOWTO_STEPS
from pracmln.mln.util import edict


        
        

class Frame(object):
    '''
    Represents a (partially) instantiated action core that is stored in the MongoDB. 
    '''
    def __init__(self, sidx, sentence, words, syntax, actioncore, actionroles):
        self.sidx = sidx
        self.sentence = sentence
        self.actionroles = actionroles
        self.actioncore = actioncore
        self.syntax = syntax
        self.words = words

    
    def __str__(self):
        return '%s [%s]' % (self.actioncore, ', '.join(['%s: %s' % (k, v) for k, v in self.actionroles.items()]))
        
    
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
        


class Howto(Frame):
    '''
    Wrapper class representing a howto in PRAC.
    '''
    def __init__(self, instr, steps, import_date=None):
        Frame.__init__(self, sidx=instr.sidx, sentence=instr.sentence, syntax=instr.syntax,
                       words=instr.words, actioncore=instr.actioncore, actionroles=instr.actionroles)
        self.steps = steps
        if import_date is None:
            self.import_date = datetime.datetime.now()
        else:
            self.import_date = import_date
        
        
    def tojson(self):
        return edict({JSON_HOWTO_IMPORT_DATE: self.import_date}) +\
               edict(Frame.tojson(self)) + edict({JSON_HOWTO_STEPS: [s.tojson() for s in self.steps]})
        
    
    @staticmethod
    def fromjson(data):
        return Howto(instr=Frame.fromjson(data), 
                     steps=[Frame.fromjson(s) for s in data.get(JSON_HOWTO_STEPS)],
                     import_date=data.get(JSON_HOWTO_IMPORT_DATE))
        

class Word(object):
    '''
    This class represents a data structure of a word in a sentence.
    Sense objects are used to determine the meaning of a sentence.
    A sense object is always included in a frame, which contains a set of senses.
    WordNet is used to assign the senses.
    '''
    def __init__(self, wid, word, widx, sense, pos, lemma, misc=None):
        self.wid = wid
        self.word = word
        self.widx = widx
        self.sense = sense
        self.pos = pos
        self.lemma = lemma
        self.misc = misc

        
    def tojson(self):
        return {constants.JSON_SENSE_WORD_ID: self.wid, 
                constants.JSON_SENSE_WORD: self.word,
                constants.JSON_SENSE_LEMMA: self.lemma,
                constants.JSON_SENSE_POS: self.pos,
                constants.JSON_SENSE_WORD_IDX: self.widx,
                constants.JSON_SENSE_SENSE: self.sense,
                constants.JSON_SENSE_MISC: self.misc}
    
    
    @staticmethod    
    def fromjson(data):
        return Word(data.get(constants.JSON_SENSE_WORD),
                    data.get(constants.JSON_SENSE_LEMMA),
                    data.get(constants.JSON_SENSE_POS),
                    data.get(constants.JSON_SENSE_SENSE),
                    data.get(constants.JSON_SENSE_WORD_IDX),
                    data.get(constants.JSON_SENSE_WORD))
    

