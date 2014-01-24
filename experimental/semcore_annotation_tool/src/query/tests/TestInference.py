import unittest
from query.inference import Inference
class TestInferenceValidateQueries(unittest.TestCase):
    
    def test_should_fail_string(self):
        """queries should be a list of string or fail."""
        
        mln = None
        evidence = None
        queries = "test"
        methods = None
        params = None
        closed_world = None
        
        self.assertRaises(TypeError, Inference(mln, evidence, queries, methods, 
                                             params, closed_world))
        

if __name__=='__main__':
    unittest.main()