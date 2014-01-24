'''
Created on Dec 4, 2012

@author: meyer
'''
import unittest
import json
from fileconversion import Conversion

class TextToAnno(unittest.TestCase):
    """Tests for the text_to_anno method."""
    
    def test_should_not_accept_empty_strings(self):
        """Test if an empty string is refused."""

        self.assertRaises(ValueError, Conversion.TextToAnno, "")
    
    def test_should_only_accept_strings(self):
        """Test if the function will only accept a string."""

        self.assertRaises(TypeError, Conversion.TextToAnno, 0)
    
    def test_should_not_be_none(self):
        """Test if the result is empyt."""
        self.assertRaises(TypeError, Conversion.TextToAnno, None)
    
    def test_should_result_not_be_none(self):
        """The result of the conversion must not be none."""
        ta = Conversion.TextToAnno("The")
        res = ta.convertedString
        self.assertIsNotNone(res)

    def test_should_result_not_be_empty(self):
        """The result of the converesion must not be empty."""
        ta = Conversion.TextToAnno("The")
        res = ta.convertedString
        self.assertIsNot(res, "")
        
    def test_should_result_be_dict(self):
        """Test that the result is a dictionary."""
        ta = Conversion.TextToAnno("The")
        res = ta.convertedString
        self.assertIsInstance(res, dict)
        
    def test_should_first_entry_textName(self):
        """Checks if the first element is called textName."""
        
        ta = Conversion.TextToAnno("The")
        res = ta.convertedString
        self.assertTrue('textName' in res.keys())
    
    def test_should_result_contain_sentences_key(self):
        """Checks if the returned dict has the sentences key."""
        
        ta = Conversion.TextToAnno("The")
        res = ta.convertedString
        self.assertTrue('sentences' in res.keys())
    
    def test_should_be_same_dictionaries(self):
        """Test for a sample dictionary if the output is correct."""
        
        sample = "Put the cup on the table."
        sample_dict = dict()
        sample_dict['textName'] = "Put the cup on the table."
        sample_dict['sentences'] = []
        sample_dict['sentences'].append(dict())
        sample_dict['sentences'][0]['id'] = 0
        sample_dict['sentences'][0]['sentence'] = sample
        
        ta = Conversion.TextToAnno(sample)
        res = ta.convertedString
        self.assertDictEqual(sample_dict, res)
        
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()