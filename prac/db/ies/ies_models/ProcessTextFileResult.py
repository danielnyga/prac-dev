'''
Created on Sep 5, 2015

@author: seba
'''
from prac.db.ies.ies_models.ProcessResultBase import ProcessResultBase
from prac.db.ies.ies_models import Constants

class ProcessTextFileResult(ProcessResultBase):
    '''
    classdocs
    '''


    def __init__(self,text_source_file):
        '''
        Constructor
        '''
        super(ProcessTextFileResult, self).__init__()
        self.text_source_file = text_source_file
        self.parsing_error_sentences_list = []
        self.no_predicate_sentences_list = []
        self.no_valid_frame_sentences_list = []
        
    
    def add_frame_builder_result(self,frame_builder_result):
        self.num_nouns += frame_builder_result.num_nouns 
        self.num_verbs += frame_builder_result.num_verbs
        self.num_adjs += frame_builder_result.num_adjs
        self.num_unk += frame_builder_result.num_unk
        self.num_assert_senses += frame_builder_result.num_assert_senses
        
        extracted_frames = frame_builder_result.frame_list
        self.num_extracted_frames += len(extracted_frames)
        
        for frame in extracted_frames:
            slot_values = frame.slot_values
            
            for slot in slot_values.keys():
                value = slot_values[slot]
                if slot in self.num_slot_values.keys():
                    if isinstance(value, list):
                        self.num_slot_values[slot] += len(value)
                    else:
                        self.num_slot_values[slot] += 1
                else:
                    if isinstance(value, list):
                        self.num_slot_values[slot] = len(value)
                    else:
                        self.num_slot_values[slot] = 1
        
    def to_json_str(self):
        json_layout="""
                    {{{{ 
                            "{}" : "{{}}",
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}},
                            "{}" : {{}}
                    }}}}
                    """.format(Constants.JSON_PROCESS_TEXT_FILE_RESULT_ID,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NO_PREDICATE_SENTENCES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NO_VALID_FRAME_SENTENCES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_PARSING_ERROR_SENTENCES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_SENTENCES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_ERRORS,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_PARSING_ERROR_SENTENCES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_NO_PREDICATE_SENTENCES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_NO_VALID_FRAME_SENTENCES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_EXTRACTED_FRAMES,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_NOUNS,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_VERBS,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_ADJS,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_UNK,
                               Constants.JSON_PROCESS_TEXT_FILE_RESULT_NUM_ASSERT_SENSES)
        
        parsing_error_sentences_json_list = "[{}]".format(', '.join(map(lambda s: s.to_json_str(), self.parsing_error_sentences_list)))
        no_predicate_sentences_json_list = "[{}]".format(', '.join(map(lambda s: s.to_json_str(), self.no_predicate_sentences_list)))
        no_valid_frame_sentences_json_list = "[{}]".format(', '.join(map(lambda s: s.to_json_str(), self.no_valid_frame_sentences_list)))
        
        result = json_layout.format(self.text_source_file,
                                    no_predicate_sentences_json_list,
                                    no_valid_frame_sentences_json_list,
                                    parsing_error_sentences_json_list,
                                    str(self.num_sentences),
                                    str(self.num_errors),
                                    str(self.num_parsing_errors_sentences),
                                    str(self.num_no_predicate_sentences), 
                                    str(self.num_no_frame_sentences), 
                                    str(self.num_extracted_frames),
                                    str(self.num_nouns),
                                    str(self.num_verbs),
                                    str(self.num_adjs),
                                    str(self.num_unk),
                                    str(self.num_assert_senses))
        
        return result
        
        
