'''
Created on Dec 4, 2012

@author: meyer
'''
import unittest
from src.ConsoleApplication import ConsoleApplication


class TestConsoleApplication(unittest.TestCase):
        
    def test_no_input(self):
        """Test for no input at all."""
        
        args = dict()
        self.assertRaises(KeyError, ConsoleApplication, args)

    def test_constructor_parameters(self):
        "Test for empty input."
        
        args = dict()
        args['mln'] = None
        args['string'] = None
        args['text'] = None
        self.assertRaises(ValueError, ConsoleApplication, args)
    
    def test_empty_string_parameters(self):
        """ test for "" as parameters. """
        
        args = dict()
        args['mln'] = ""
        args['string'] = ""
        args['text'] = ""
        self.assertRaises(ValueError, ConsoleApplication, args)
    
    def test_correct_input_s(self):
        """Test if model and sting are present and correct."""
        
        args = dict()
        args['mln'] = "/home/meyer/workspace/prac/semcore_annotation_tool/" \
                    "models/experiments/grounding/put/11/learned_model.mln"
        args['string'] = "Put the cup on the table."
        args['text'] = None
        ca = ConsoleApplication(args)
        self.assertIsInstance(ca, ConsoleApplication)
        
    def test_mutual_parameter_exclusion(self):
        """Test if mutual exclusion of parameters is correct."""
        
        args = dict()
        args['mln'] = "test"
        args['string'] = "test"
        args['text'] = "test"
        self.assertRaises(ValueError, ConsoleApplication, args)
    
    def test_both_Parameters_missing(self):
        """Test if fails if both important parameters are missing."""
        
        args= dict()
        args['mln'] = "test"
        args['string'] = None
        args['text'] = None
        self.assertRaises(ValueError, ConsoleApplication, args)
    
    def test_non_existing_file(self):
        """Test for non existing file."""
        
        args = dict()
        args['mln'] = "/home/meyer/workspace/prac/semcore_annotation_tool/" \
                    "models/experiments/grounding/put/11/learned_model.mlnsns"
        args['string'] = "Put the cup on the table."
        args['text'] = None
        self.assertRaises(ValueError, ConsoleApplication, args)
    
    def test_should_return_correct_text_type_string(self):
        """Check if the function returns the correct text type."""
        
        args = dict()
        args['mln'] = "/home/meyer/workspace/prac/semcore_annotation_tool/" \
                    "models/experiments/grounding/put/11/learned_model.mln"
        args['string'] = "Put the cup on the table."
        args['text'] = None
        ca = ConsoleApplication(args)
        res = ca._check_parameter(args)
        
        self.assertEqual('string', res)
    
    def test_should_return_correct_text_type_file(self):
        """Check if the function returns the correct text type."""
        
        args = dict()
        args['mln'] = "/home/meyer/workspace/prac/semcore_annotation_tool/" \
                    "models/experiments/grounding/put/11/learned_model.mln"
        args['string'] = None
        args['text'] = "/home/meyer/workspace/prac/semcore_annotation_tool/" \
                        "sample_data/ehow_files/test.ehow"
        ca = ConsoleApplication(args)
        res = ca._check_parameter(args)
        
        self.assertEqual('text', res)
        
        
if __name__ == "__main__":
    unittest.main()