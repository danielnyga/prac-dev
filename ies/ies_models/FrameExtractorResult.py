'''
Created on Sep 5, 2015

@author: seba
'''
from ies_models.ProcessResultBase import ProcessResultBase

class FrameExtractorResult(ProcessResultBase):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        super(FrameExtractorResult, self).__init__()
        self.process_text_file_result_list = []
        self.frame_list = []
    
    def add_process_text_file_result(self,process_text_file_result):
        self.process_text_file_result_list.append(process_text_file_result)
        
        self.num_nouns += process_text_file_result.num_nouns 
        self.num_verbs += process_text_file_result.num_verbs
        self.num_adjs += process_text_file_result.num_adjs
        self.num_unk += process_text_file_result.num_unk
        self.num_assert_senses += process_text_file_result.num_assert_senses
        
        self.num_sentences += process_text_file_result.num_sentences
        self.num_extracted_frames += process_text_file_result.num_extracted_frames
        self.num_errors += process_text_file_result.num_errors
        self.num_parsing_errors_sentences += process_text_file_result.num_parsing_errors_sentences
        self.num_no_predicate_sentences += process_text_file_result.num_no_predicate_sentences
        self.num_no_frame_sentences += process_text_file_result.num_no_frame_sentences
        
        
        if self.num_slot_values:
            slot_values = process_text_file_result.num_slot_values
            
            for slot in slot_values.keys():
                value = slot_values[slot]
                
                if slot in self.num_slot_values.keys():
                    self.num_slot_values[slot] += value
                else:
                    self.num_slot_values[slot] = value
        else:
            self.num_slot_values = process_text_file_result.num_slot_values
    
    def merge_frame_extractor_result(self,frame_extractor_result):
        
        self.process_text_file_result_list.extend(frame_extractor_result.process_text_file_result_list)
        
        self.frame_list.extend(frame_extractor_result.frame_list)
        self.num_nouns += frame_extractor_result.num_nouns 
        self.num_verbs += frame_extractor_result.num_verbs
        self.num_adjs += frame_extractor_result.num_adjs
        self.num_unk += frame_extractor_result.num_unk
        self.num_assert_senses += frame_extractor_result.num_assert_senses
        
        self.num_sentences += frame_extractor_result.num_sentences
        self.num_extracted_frames += frame_extractor_result.num_extracted_frames
        self.num_errors += frame_extractor_result.num_errors
        self.num_parsing_errors_sentences += frame_extractor_result.num_parsing_errors_sentences
        self.num_no_predicate_sentences += frame_extractor_result.num_no_predicate_sentences
        self.num_no_frame_sentences += frame_extractor_result.num_no_frame_sentences
        
        
        if self.num_slot_values:
            slot_values = frame_extractor_result.num_slot_values
            
            for slot in slot_values.keys():
                value = slot_values[slot]
                
                if slot in self.num_slot_values.keys():
                    self.num_slot_values[slot] += value
                else:
                    self.num_slot_values[slot] = value
        else:
            self.num_slot_values = frame_extractor_result.num_slot_values