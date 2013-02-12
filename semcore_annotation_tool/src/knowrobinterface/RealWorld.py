import os
import roslib
roslib.load_manifest('json_prolog')
import rospy
import json_prolog
import yaml
import sys
import actions

class Mapping:
    
    def __init__(self):
        self._load_semantic_map()
        self._kr_objects = self._load_objects_from_file()
        self._kr_concepts = self.query_knowrob_for_concepts_of_objects(
                                                            self._kr_objects)

    def load_relations_kr_mln_mapping(self, f="RelationsMapping.yaml"):
        """Load the mapping of kr relations to mln predicates."""
        
        dir = os.path.dirname(os.path.abspath(__file__))
        file_name = os.path.join(dir, f)
        try:
            with open(file_name,'r') as f:
                relations_mapping = yaml.load(f)
        except Exception:
            print "error loading objects from yaml"
            sys.exit(0)
        return relations_mapping
    
    def get_kr_instances(self):
        return self._kr_objects
    
    def get_nltk_concepts(self):
        """Get the nltk concepts for the world defined in the config."""
        
        return self.get_nltk_mappings_for_kr_objects(self._kr_concepts)
    
    def _load_semantic_map(self):
        """ Load the semantic map for the queries."""
        prolog = json_prolog.Prolog()
        rospy.init_node("annotationTool_node")
        florian_kitchen = os.path.join(
                                os.path.dirname(os.path.abspath(__file__)),
                                '..','..','..','semantic_maps',
                                'florian_kitchen.owl') 
        
        query_string = "owl_parse('{map}', false, false, true)".format(
                                                    map=florian_kitchen)
        res = prolog.query(query_string)
        res.finish()
        
    def _load_objects_from_file(self, f="Objects.yaml"):
        dir = os.path.dirname(os.path.abspath(__file__))
        file_name = os.path.join(dir, f)
        try:
            with open(file_name,'r') as f:
                kr_objects = yaml.load(f)
        except Exception:
            print "error loading objects from yaml"
            sys.exit(0)
        return kr_objects
    
    def get_nltk_mappings_for_kr_objects(self, kr_concepts):
        """Get the mappings to NLTK for all the provided KR concepts."""
        
        currrent_dir = os.path.dirname(os.path.abspath(__file__))
        mappings_yaml_f = os.path.join(currrent_dir,'..','..','..',
                                       'mappings','ontology_mapping.yaml')
        
        try:
            with open(mappings_yaml_f,'r') as f:
                nltk_kr_mapping = yaml.load(f)
        except Exception as e:
            print("The NLTK KR mapping could not be loaded")
            print e
            sys.exit(0)
        nltk_objects = dict()
        
        for individual, kr_concept_dict_list in kr_concepts.iteritems():
            item_name = individual.split('#',1)[1]
            nltk_objects[item_name] = []
            for kr_concept in kr_concept_dict_list:
                if kr_concept['A'] in nltk_kr_mapping:
                    nltk_objects[item_name].append(
                                            nltk_kr_mapping[kr_concept['A']])
                
        return nltk_objects
    
    def query_knowrob_for_concepts_of_objects(self, objects):
        """Get the KnowRob concepts for each object."""
        
        prolog = json_prolog.Prolog()
        rospy.init_node("annotationTool_node")
        concepts = {}
        for obj in objects:
            query_string = "owl_has('{obj}',rdf:type,A)".format(obj=obj)
            
            res = prolog.query(query_string)
            concepts[obj] = [x for x in res.solutions()]
            res.finish()
        return concepts
    
class Relations:
    def __init__(self):
        self._relations = self._load_relationships_from_yaml()
    
    def __str__(self):
        """MLN representation of the relations."""
        mapp = Mapping()
        map = mapp.load_relations_kr_mln_mapping()
        relationships = self.get_relationship(mapp.get_kr_instances())
        res = []
        for obj1_kr, rels in relationships.iteritems():
            obj1 = obj1_kr.split("#", 1)[1]
            for kr_rel, obj2_list in rels.iteritems():
                if kr_rel in self._relations:
                    for obj2_kr in obj2_list:
                        obj2 = obj2_kr['A'].split("#",1)[1]
                        res.append("{rel}({obj1},{obj2})".format(
                                        rel=map[kr_rel], obj1=obj1, obj2=obj2))
        
        return "\n".join(res)
    
    def _load_relationships_from_yaml(self, f="ObjectRelation.yaml"):
        """
        
        Load the realtionships that are to be considered from the 
        config file.
        
        """
        
        dir = os.path.dirname(os.path.abspath(__file__))
        file_name = os.path.join(dir, f)
        try:
            with open(file_name,'r') as f:
                relations = yaml.load(f)
        except Exception:
            print "error loading objects from yaml"
            sys.exit(0)
        return relations
    
    def get_relationship(self, kr_instances):
        """Get the relationship between the knowrob instances."""
        relationship = {}
        relationship = dict(relationship.items() + 
                            self._query_kr_for_instance_relations(kr_instances).items())
        return relationship
    
    def _query_kr_for_instance_relations(self, kr_instances):
        """Get the KnowRob concepts for each object."""
        
        prolog = json_prolog.Prolog()
        rospy.init_node("annotationTool_node")
        relations = {}
        for kr_instance in kr_instances:
            relations[kr_instance] = {}
            for relation in self._relations:
                relations[kr_instance][relation] = []
                query_string = "owl_has('{inst}',{rel},A)".format(
                                                        inst=kr_instance,
                                                        rel = relation)
            
                res = prolog.query(query_string)
                relations[kr_instance][relation]+=[x for x in res.solutions()]
                res.finish()
        return relations
    
    def _generate_MLN_from_relations(self, relations):
        """Create the MLN representation of the relations."""
        
        mapp = Mapping()
        mapping = mapp.load_relations_kr_mln_mapping()
        
        return mapping
    
if __name__=="__main__":
    mapp = Mapping()
    rel = Relations()
    instances = mapp.get_kr_instances()
    concepts =  mapp.get_nltk_concepts()
    relations = rel.get_relationship(instances)
    print(rel)
