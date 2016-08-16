'''
This class represents a data structure of a word in a sentence.
Sense objects are used to determine the meaning of a sentence.
A sense object is always included in a frame, which contains a set of senses.
WordNet is used to assign the senses.

Created on Sep 2, 2015

@author: Sebastian Koralewski (seba@informatik.uni-bremen.de)
'''
import json
from nltk.stem import WordNetLemmatizer
from prac.core.wordnet import WordNet
from prac.db.ies.ies_models import Constants

wordnet = WordNet(concepts=None)
nounTags = ['NN', 'NNS', 'NNP', 'CD']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
adjTags = ['JJ', 'JJR', 'JJS']

posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'
for a in adjTags:
    posMap[a] = 'a'

class Sense(object):
    '''
    classdocs
    '''


    def __init__(self,word,penn_treebank_pos,nltk_wordnet_sense="",wordnet_pos="",lemma="",misc="",process_sense=True):
        '''
        Constructor
        '''
        
        self.word = word
        self.penn_treebank_pos = penn_treebank_pos
        self.misc = misc
        
        if process_sense:
            self.wordnet_pos = convert_penn_treebank_pos_to_wordnet_pos(penn_treebank_pos)
            self.lemma = convert_word_to_lemma(word, penn_treebank_pos)
         
            if nltk_wordnet_sense: 
                self.nltk_wordnet_sense = nltk_wordnet_sense
            else:
                self.nltk_wordnet_sense = try_to_determine_wordnet_sense(self.lemma, self.wordnet_pos)
        else:
            self.nltk_wordnet_sense = nltk_wordnet_sense
            self.wordnet_pos = wordnet_pos
            self.lemma = lemma
    def to_json_str(self):
        json_layout = """{{{{
                            "{}" : "{{}}", 
                            "{}" : "{{}}",
                            "{}" : "{{}}", 
                            "{}" : "{{}}",
                            "{}" : "{{}}", 
                            "{}" : "{{}}" 
                        }}}} 
                        """.format(Constants.JSON_SENSE_WORD,
                                       Constants.JSON_SENSE_LEMMA,
                                       Constants.JSON_SENSE_PENN_TREEBANK_POS,
                                       Constants.JSON_SENSE_WORDNET_POS,
                                       Constants.JSON_SENSE_NLTK_WORDNET_SENSE,
                                       Constants.JSON_SENSE_MISC)
        
        sense_as_json_str = json_layout.format(self.word,
                                           self.lemma,
                                           self.penn_treebank_pos,
                                           self.wordnet_pos,
                                           self.nltk_wordnet_sense,
                                           self.misc)
        
        return sense_as_json_str

def convert_word_to_lemma(word,penn_treebank_pos):
    wnl = WordNetLemmatizer()
    word_ = '-'.join(word.split('-')[:-1])
    
    #In case of that the word is given in the form word instead word-1
    if not word_:
        word_ = word
    #TODO add NNP check to avoid treating regular nouns as proper nouns. 
    #Proper nouns should be kept unchanged
    if penn_treebank_pos == 'NNP':
        return word_
    else:
        word_ = word_.lower()
    
    wn_pos = convert_penn_treebank_pos_to_wordnet_pos(penn_treebank_pos)
    
    if wn_pos == 'unk':
        return word_
    
    return str(wnl.lemmatize(word_,wn_pos))

def convert_penn_treebank_pos_to_wordnet_pos(penn_treebank_pos):
    return posMap.get(penn_treebank_pos, "unk")
    
#if the word have just one synset, we can assign this sense to the word.
def try_to_determine_wordnet_sense(lemma,wordnet_pos):
    syns = get_synset(lemma, wordnet_pos)
    
    if len(syns) == 1:
        return syns[0].name
    else:
        return ""

def get_synset(lemma,wordnet_pos):
    if wordnet_pos == 'unk':
        return []
    return wordnet.synsets(lemma,wordnet_pos)