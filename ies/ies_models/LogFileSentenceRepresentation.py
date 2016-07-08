'''
Created on Sep 9, 2015

@author: seba
'''

from ies_models import Constants
import cStringIO

class LogFileSentenceRepresentation(object):
    '''
    classdocs
    '''


    def __init__(self, sentence,mln,db):
        '''
        Constructor
        '''
        self.sentence = sentence
        self.prac_mln = mln
        self.prac_db = db
    
    def to_json_str(self):
        json_layout = """
                    {{{{
                        "{}" : "{{}}",
                        "{}" : "{{}}",
                        "{}" : "{{}}"
                    }}}}
                    """.format(Constants.JSON_LOG_FILE_SENTENCE_REPRESENTATION_SENTENCE,
                               Constants.JSON_LOG_FILE_SENTENCE_REPRESENTATION_PRAC_MLN,
                               Constants.JSON_LOG_FILE_SENTENCE_REPRESENTATION_PRAC_DB)
        prac_mln_stream = cStringIO.StringIO()
        prac_db_stream = cStringIO.StringIO()
        prac_mln_str = ""
        prac_db_str = ""
        
        try:
            if self.prac_db is not None:
                self.prac_db.write(prac_db_stream)
                prac_db_stream.seek(0)
                self.prac_mln.write(prac_db_stream)
                prac_mln_stream.seek(0)
                prac_mln_str = prac_mln_stream.read().encode('string-escape')
                prac_db_str = prac_db_stream.read().encode('string-escape')
            
        except:
            print "ERROR during convert LogFileSentence to JSON"
            
        result = json_layout.format(self.sentence,
                           prac_mln_str,
                           prac_db_str)
        
        return result
