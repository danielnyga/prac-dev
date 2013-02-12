'''
Created on Dec 4, 2012

@author: meyer
'''
import argparse
import utils
import os
import sys
import re
from Visualization import Visualization
import yaml
import annotation.publicInterface
from annotation.publicInterface import annotationInterface
from query.inference import Inference
class ConsoleApplication:
    """
    Parse a text and present the user with a visual output in the knowrob
    mod_vis package.
    
    Usage:
    Optionally specify an mln that is used to process the text and infer the
    objects mentioned.
    
    Use either a text string as input or a file that has the ending .ehow which
    is plain text.
    
    """
    

    def _check_parameter(self, arguments):
        """
        Check the parameters provided to the class and raise an
        exception if there is a problem.
        
        Returns:
            - key in the arguments dictionary for the type of text input
        """
        
        text_type = None
        if not "mln" in arguments or (not "string" in arguments or 
            not "text" in arguments or not "database"):
            raise (KeyError)
        if not utils.check_string_not_none_special_empty(arguments['mln']):
            raise (ValueError)
        if utils.check_string_not_none_special_empty(arguments['string']):
            if utils.check_string_not_none_special_empty(arguments['text']):
                raise (ValueError)
            text_type = 'string'
        if utils.check_string_not_none_special_empty(arguments['text']):
            if utils.check_string_not_none_special_empty(arguments['string']):
                raise (ValueError)
            text_type = 'text'
        if utils.check_string_not_none_special_empty(arguments['database']):
            text_type = 'database'
        else:
            raise (ValueError)
        if not os.path.exists(arguments['mln']):
            print "The provided MLN does not exits."
            raise(ValueError)
        assert(isinstance(text_type, str))
        
        return text_type

    def _load_config(self):
        """Load the config from the config.yaml file in the package folder."""
        
        with open("ConsoleApplicationConfig.yaml",'r') as f:
            loaded_conf = yaml.load(f)
            print("loaded config")
        f.close()
        config = dict()
        for con in loaded_conf:
            for key, item in con.iteritems():
                config[key] = item
        return config
    
    def _load_evidence_db(self, file_path):
        """Load an evidence DB from .db file."""
        
        db_file = utils.load_file_return_string(file_path)
        return db_file
        
    
    def __init__(self, args):
        #raises an exception if there is a problem
        text_type = self._check_parameter(args)
        self.config = self._load_config()
        if text_type == 'text':
            #load text from file
            raise(NotImplementedError)
        elif text_type == 'database':
            evidence = self._load_evidence_db(args['database'])
            if evidence == "":
                sys.exit(1)

        else:
            text = args[text_type]
            self.anno = annotationInterface(texts = [text])
            assert(isinstance(self.anno, annotationInterface))
            #we can only query one text per time -> we can pick the 1st element
            evidence = str(self.anno.getMLN(realWorld=true)[0])
        assert(isinstance(evidence, str))
        assert(not evidence=="")
        
        #now get the mln
        mln = utils.load_file_return_string(args['mln'])
        if len(mln) == 0:
            raise(IOError)
            return
        assert(len(mln)>0)
                
        query_predicates = ",".join(self.config['query'])
        assert(isinstance(query_predicates, str))
        
        mln_inference = Inference(mln, evidence, query_predicates,
                                  self.config['closed_world'])
        
        results = mln_inference.run()
        grouding_results = self._filter_results(results, "isGrounded")
        instances = self._extract_instances(grounding_results)
        
        #get the mappign!
        viz = Visualization()
        print(results)
    
    def _extract_instances(self, grounding_results):
        """Extract the instances that are returned in a result set."""
        
        result = {}
        for res in grouding_results:
            tmp_result= re.findall(r"[a-zA-Z0-9_]+", res)
            result[tmp_result[1]] = tmp_result[2]
            
        return result
            
    
    def _filter_results(self, results, filter):
        """Filter a list of results to get only the desired."""
        
        pattern = "^{filter}.+$"
        res_list = []
        for res in results:
            if not re.search(patter, res) is None:
                res_list.append(res)
        return res_list
        
        return res
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.description = description="""Parse a text and find objects mentioned 
                                        in the semantic map of the ROS KnowRob 
                                        package. Visualizes the found objects
                                        and return the identifier."""
    group = parser.add_mutually_exclusive_group()
    parser.add_argument("-m", "--mln", 
                                help="the mln to find objects in the map")
    group.add_argument("-t", "--text", help="a text file to be parsed")
    group.add_argument("-s", "--string", help="a text string to be parsed")
    group.add_argument("-db", "--database", help="a Markov Logic database")
    
    args = parser.parse_args()
    
    if not (args.mln and (args.string or args.database or args.text)): 
        print "Usage: {prog_name} -m {mln_file} -t/-s/-db "\
        "{text_db_string}".format(prog_name = __file__, 
                                  mln_file="'Path to MLN File'", 
                                  text_db_string="a sentence/path to text "\
                                                "file/path to db file")
        sys.exit(1)
        
    
    try:
        ca = ConsoleApplication(vars(args))
    except ValueError as e:
        print(e)
        