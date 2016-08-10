from scipy import stats
from prac.core.wordnet import WordNet


def frame_similarity(frame_1_actionroles_dict, frame_2_actionroles_dict):
    '''
    Determines the frame similarity between two given frames.
    The frame similarity is calculated by taking the harmonic mean of the actionroles between the given frames.
    This value can be interpreted as the semantic similarity between the frames.
      
    Is is required that the actionroles of the corresponding frames are represented as a dictionary.
    The dictionary must have the form: role_name : nltk_wordnet_sense

    :param frame_1_actionroles_dict: Represents the actionroles contained in frame 1
    :param frame_2_actionroles_dict: Represents the actionroles contained in frame 2
    :return: The frame similarity between frame 1 and frame 2.
    '''
    wordnet = WordNet(concepts=None)
    frame_vector = []
    is_frame_inconsistent = False
    action_verb_sim = wordnet.wup_similarity(str(frame_2_actionroles_dict['action_verb']),
                                             str(frame_1_actionroles_dict['action_verb']))
    
    #This is a sanity check to revoke false inferred frames during the information extraction process.
    if action_verb_sim  < 0.85:
        return 0
     
    for role, sense in frame_1_actionroles_dict.iteritems():
        if role in frame_2_actionroles_dict.keys():
            sim = wordnet.wup_similarity(str(frame_2_actionroles_dict[role]), sense)
            #Sometimes Stanford Parser parses some objects as adjectives
            #due to the fact that nouns and adjectives cannot be compared
            #we define the the similarity between the instruction and the frame as zero
            if sim == 0:
                is_frame_inconsistent = True
            else:
                frame_vector.append(wordnet.wup_similarity(str(frame_2_actionroles_dict[role]), sense))
    
    if is_frame_inconsistent:
        return 0
    
    return stats.hmean(frame_vector)