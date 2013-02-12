'''
Created on Dec 4, 2012

@author: meyer
'''
import unittest
import utils

class TestUtils(unittest.TestCase):

    def test_check_string_not_none_special_empty_none(self):
        """Test for None input """
        
        arg = None
        res = utils.check_string_not_none_special_empty(arg)
        self.assertFalse(res)
        
    def test_check_string_not_none_empty_empty(self):
        """ Test for empty input """
        
        arg = ""
        res = utils.check_string_not_none_special_empty(arg)
        self.assertFalse(res)
    
    def test_check_string_not_none_empty_special(self):
        """ Test for special chars as input. """
        
        arg = "@#$%^&*()+?><\\\/////`="
        for c in arg:
            res = utils.check_string_not_none_special_empty(c)
            self.assertFalse(res)
        
if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()