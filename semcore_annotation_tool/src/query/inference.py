'''
Created on Aug 23, 2012

@author: meyer
'''
import yaml
import tempfile
import os
from mlnQueryTool import MLNInfer
class Inference:

    def __init__(self, mln_formulae, evidence, queries, 
                closed_world, dir=os.getcwd()):
        """
        Inference class that provides functionality to run inference over
        evidence.
        
        Parameters:
            - mln: string representation of an MLN
            - evidence: string representation of an evidence DB
            - queries: list of query predicates as comma seperated strings
            - methods: methods and the engines to be used for the inference
            - closed_word:list of predicates where the closed world
                          assumption holds.
  
        """
        self.queries = queries
        self.evidence = evidence
        self.mln_formulae = mln_formulae
        self.closed_world = closed_world
        self.parameters = {}
        self._export_dir = dir 
        self.methods = dict()
        self._load_config()
        self._temp_files = []
    
    def _load_config(self):
        """Load the config from the config.yaml file in the package folder."""
        
        config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "config.yaml")
        with open(config_file,'r') as f:
            config = yaml.load(f)
            print("loaded config")
        f.close()
        #load parameters:
        for par in config['parameters']:
            for key, value in par.iteritems():
                self.parameters[key] = value
        for engine, methods in config['methods'].iteritems():
            for method in methods:
                self.methods[engine] = method
    
    def _create_temp_file_with_data(self,data, dir = None):
        """
        
        Create a temporary file and fill it with the string represenation
        of the data provided. Data is a list of strings. Directory is
        optional.
        Return the path to the temp file.
        
        """
        fd, file_name = tempfile.mkstemp('.tmp', dir=dir)
        self._temp_files.append(file_name)
        if not data is None:
            with open(file_name,'w+') as f:
                f.write("{data}".format(data = data))
            f.close()
        return file_name
    
    def _delete_temp_files(self):
        """
        
        Delete all temporary files that were created with the 
        _create_temp_file_with_data method.
        
        """
        
        if not self._temp_files is None:
            if len(self._temp_files) > 0:
                for f in self._temp_files:
                    try:
                        os.remove(f)
                    except Exception:
                        continue
    
    def run(self):
        """Run inference for this inference object."""
        
        evidence_temp_abs_filename = self._create_temp_file_with_data(
                                                        self.evidence,
                                                        dir = self._export_dir)
        evidence_temp_file = evidence_temp_abs_filename.rsplit('/',1)[1]
        
        mln_temp_abs_filename = self._create_temp_file_with_data(
                                                        self.mln_formulae,
                                                        dir = self._export_dir)
        mln_temp_file = mln_temp_abs_filename.rsplit('/',1)[1]
        output_temp_abs_filename = self._create_temp_file_with_data(None, 
                                                        dir = self._export_dir)
        output_temp_filename = output_temp_abs_filename.rsplit('/',1)[1]
        
        infer = MLNInfer()
        results = {}
        method_list = []
        try:
            for engine, methods in self.methods.iteritems():
                if isinstance(methods, str):
                    method_list.append(methods)
                else:
                    method_list += methods
                for method in method_list: 
                    results[(method,engine)] = infer.run([mln_temp_file], 
                                    evidence_temp_file,
                                    method,
                                    self.queries,
                                    engine,
                                    output_filename = output_temp_filename, 
                                    saveResults=self.parameters['saveResults'],
                                    maxSteps=self.parameters['maxSteps'])
        except Exception as e:
            self._delete_temp_files()
            print(e)
            return False
        self._delete_temp_files
        return results

        
if __name__=="__main__":
    mln_file = "/home/meyer/workspace/prac/semcore_annotation_tool/models/" \
                "experiments/grounding/put/11/learned_model.mln"
    evidence = "hasPOS(Hallo, NN)"
    i = Inference("hasPOS(word,pos)\n0.5 hasPOS(x,w)", 
                  evidence,
                  "hasPOS", 
                  {"cwPreds": "hasPOS, dobj"},
                  dir = None)
    i.run()