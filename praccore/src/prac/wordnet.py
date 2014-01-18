# PROBABILISTIC ROBOT ACTION CORES 
#
# (C) 2013 by Daniel Nyga (nyga@cs.tum.edu)
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
from nltk.corpus import wordnet
from pracutils.graph import DAG , Node
import nltk
import logging
import pickle
from itertools import chain
from nltk.corpus.reader.wordnet import Synset
from utils.graphml import Graph, Node as GMLNode, Edge
import sys
# nltk.data.path = [os.path.join('data', 'nltk_data')]
# wordnet_data_path = os.path.join('data', 'wordnet')
nltk.data.path = [os.path.join('data', 'nltk_data')]
wordnet_data_path = os.path.join('data', 'wordnet')

NLTK_POS = ['n', 'v', 'a', 'r']

class WordNet(object):
    '''
    Wrapper class for WordNet, which may be initialized with
    some WordNet concepts spanning an initial, collapsed
    concept taxonomy.
    '''
    
    def __init__(self, concepts=None):
        self.core_taxonomy = None
        if concepts is not None:
            self.initialize_taxonomy(concepts)
        self.subtree_cache_path = os.path.join(wordnet_data_path, 'subtrees')
        self.dirty_cache = False
        if os.path.exists(self.subtree_cache_path):
            self.subtree_cache = pickle.load(open(self.subtree_cache_path, 'r'))
        else:
            self.subtree_cache_path = os.path.join('.', 'wordnet.cache') 
            self.subtree_cache = {}

            
    def initialize_taxonomy(self, concepts=None, collapse=True):
        '''
        Creates a new taxonomy given a set of concepts. If collapse is True,
        all subpaths with only one child and parent are collapsed. 
        '''
        log = logging.getLogger('WordNet')
        entity_name = 'entity.n.01'
        self.core_taxonomy = DAG(root=Node(entity_name, entity_name))
        self.known_concepts = [entity_name]
        if concepts is None:
            self.core_taxonomy = None
        else:
            for concept in concepts:
                log.debug(concept)
                if concept == 'NULL': continue
                synset = wordnet.synset(concept)
                paths = synset.hypernym_paths()
                for path in paths:
                    WordNet.__extend_taxonomy_graph(self.core_taxonomy.root, path[1:])
            # collapse the taxonomy
            queue = list(self.core_taxonomy.root.children)
            while len(queue) > 0:
                node = queue.pop()
                if collapse:
                    keepThisNode = len(node.children) == 0 or len(node.children) > 1
                    keepThisNode |= len(node.parents) > 1 
                    keepThisNode |= len(node.siblings()) >= 1
#                     keepThisNode |= node.data in concepts
                    for c in node.children:
                        keepThisNode |= len(c.parents) > 1
                    if not keepThisNode:
                        for p in node.parents:
                            p.children.remove(node)
                            p.children.update(node.children)
                        for c in node.children:
                            c.parents.remove(node)
                            c.parents.update(node.parents)
                    else:
                        self.known_concepts.append(node.data)
                else:
                    self.known_concepts.append(node.data)
                queue.extend(node.children)
    
    def synsets(self, word, pos):
        '''
        Returns the set of synsets from NLTK.
        - word:     (string) the word to be queried.
        - pos:      (string) the NLTK POS tag.
        '''
        if not pos in NLTK_POS:
            logging.getLogger('WordNet').exception('Unknown POS tag: %s' % pos)
        synsets = wordnet.synsets(word, pos)
        if self.core_taxonomy is not None:
            synsets = filter(lambda s: s.name in self.known_concepts, synsets)
        return synsets


    def synset(self, synset_id):
        '''
        Returns the NLTK synset for the given id.
        '''
        if synset_id == 'NULL':
            return None
        try: 
            synset = wordnet.synset(synset_id)
            return synset
        except Exception, e:
            logging.getLogger('WordNet').error('Could not obtain synset with ID "%s"' % synset_id)
            raise e
        
    def wup_similarity(self, synset1, synset2):
        '''
        Returns the WUP similariy of the two given synsets, which
        may be given as strings of the synset id or the respective Synset objects themselves.
        '''
        if type(synset1) is str:
            synset1 = self.synset(synset1)
        if type(synset2) is str:
            synset2 = self.synset(synset2)
        if synset1 is None or synset2 is None:
            return 0.
        similarity = synset1.wup_similarity(synset2)
#         similarity = synset1.path_similarity(synset2)
        return 0. if similarity is None else similarity
    
    def lowest_common_hypernyms(self, synset, other, simulate_root=False, use_min_depth=False):
        """
        -- NOTE: THIS CODE IS COPIED FROM NLTK3 --
        Get a list of lowest synset(s) that both synsets have as a hypernym.
        When `use_min_depth == False` this means that the synset which appears as a
        hypernym of both `self` and `other` with the lowest maximum depth is returned
        or if there are multiple such synsets at the same depth they are all returned

        However, if `use_min_depth == True` then the synset(s) which has/have the lowest
        minimum depth and appear(s) in both paths is/are returned.

        By setting the use_min_depth flag to True, the behavior of NLTK2 can be preserved.
        This was changed in NLTK3 to give more accurate results in a small set of cases,
        generally with synsets concerning people. (eg: 'chef.n.01', 'fireman.n.01', etc.)

        This method is an implementation of Ted Pedersen's "Lowest Common Subsumer" method
        from the Perl Wordnet module. It can return either "self" or "other" if they are a
        hypernym of the other.

        :type other: Synset
        :param other: other input synset
        :type simulate_root: bool
        :param simulate_root: The various verb taxonomies do not
            share a single root which disallows this metric from working for
            synsets that are not connected. This flag (False by default)
            creates a fake root that connects all the taxonomies. Set it
            to True to enable this behavior. For the noun taxonomy,
            there is usually a default root except for WordNet version 1.6.
            If you are using wordnet 1.6, a fake root will need to be added
            for nouns as well.
        :type use_min_depth: bool
        :param use_min_depth: This setting mimics older (v2) behavior of NLTK wordnet
            If True, will use the min_depth function to calculate the lowest common
            hypernyms. This is known to give strange results for some synset pairs
            (eg: 'chef.n.01', 'fireman.n.01') but is retained for backwards compatibility
        :return: The synsets that are the lowest common hypernyms of both synsets
        """

        fake_synset = Synset(None)
        fake_synset._name = '*ROOT*'
        fake_synset.hypernyms = lambda: []
        fake_synset.instance_hypernyms = lambda: []

        if simulate_root:
            self_hypernyms = chain(synset._iter_hypernym_lists(), [[fake_synset]])
            other_hypernyms = chain(other._iter_hypernym_lists(), [[fake_synset]])
        else:
            self_hypernyms = synset._iter_hypernym_lists()
            other_hypernyms = other._iter_hypernym_lists()

        synsets = set(s for synsets in self_hypernyms for s in synsets)
        others = set(s for synsets in other_hypernyms for s in synsets)
        if self.core_taxonomy is not None:
            synsets.intersection_update(map(lambda syn: wordnet.synset(syn), self.known_concepts))
            others.intersection_update(map(lambda syn: wordnet.synset(syn), self.known_concepts))
        synsets.intersection_update(others)

        try:
            if use_min_depth:
                max_depth = max(s.min_depth() for s in synsets)
                unsorted_lch = [s for s in synsets if s.min_depth() == max_depth]
            else:
                max_depth = max(s.max_depth() for s in synsets)
                unsorted_lch = [s for s in synsets if s.max_depth() == max_depth]
            return sorted(unsorted_lch)
        except ValueError:
            return []
    
    def semilarity(self, synset1, synset2):
        '''
        Returns the our custom semantic similarity by Daniel Nyga and Dominik
        Jain of the two concepts.
        '''
        if type(synset1) is str:
            synset1 = self.synset(synset1)
        if type(synset2) is str:
            synset2 = self.synset(synset2)
        if synset1 is None or synset2 is None:
            return 0.
        if synset1 == synset2:
            return 1.0
        h_r = self.get_subtree_height('entity.n.01')
        h_a = self.get_subtree_height(synset1)
        h_b = self.get_subtree_height(synset2)
        s = self.lowest_common_hypernyms(synset1, synset2, use_min_depth=True, simulate_root=True)[0]
        if s.name is None: return 0.
        h_s = self.get_subtree_height(s)
        return (h_r - h_s) / (h_r - .5 * (h_a + h_b))
    
    def get_subtree_height(self, synset):
        '''
        Returns the height of the subtree of the given synset.
        '''
        if type(synset) is str:
            synset = self.synset(synset)
        if synset.name in self.subtree_cache:
            return self.subtree_cache[synset.name]
        else:
            height = self.__get_subtree_height(synset.name)
            self.subtree_cache[synset.name] = height
            f = open(self.subtree_cache_path, 'w+')
            pickle.dump(self.subtree_cache, f)
            f.close()
            return height
        
        
    def __get_subtree_height(self, synset, current_height=0):
        hypos = self.synset(synset).hyponyms()
        if self.core_taxonomy is not None:
            hypos = set(map(lambda s: s.name, hypos)).intersection(self.known_concepts)
        if len(hypos) == 0: # we have a leaf node
            return current_height
        children_heights = []
        for child in hypos:
            children_heights.append(self.__get_subtree_height(child, current_height+1))
        return max(children_heights)
        
        
    def hypernym_paths(self, synset):
        '''
        Returns a list of lists specifying the hypernymy paths
        for a given synset, just like the native NLTK function does,
        but this version uses the collapsed taxonomy instead.
        - synset:    a string of a synset id or a synset object itself.
        '''
        if type(synset) is str:
            synset = self.synset(synset)
            if synset is None: return None
        if self.core_taxonomy is None:
            return synset.hypernym_paths()
        paths = []
        for path in synset.hypernym_paths():
            new_path = []
            for concept in path:
                if concept.name in self.known_concepts:
                    new_path.append(concept)
            if not new_path in paths:
                paths.append(new_path)
        return paths
    
    
    @staticmethod
    def __extend_taxonomy_graph(concept, synset_path):
        '''
        Takes a node of the taxonomy graph and a hypernymy path
        of a concept and extends the graph with the given path.
        - concept:     (Node) concept node to be extended (the root in most cases)
        - synset_path: a path as it is return by Synset.hypernym_paths(), for instance.
        '''
        if len(synset_path) == 0:
            return
        synset = synset_path[0]
        hyponyms = synset_path[1:]
        for hyponym_node in concept.children:
            if hyponym_node.id == synset.name:
                WordNet.__extend_taxonomy_graph(hyponym_node, hyponyms)
                return
        node = Node(synset.name, synset.name)
        concept.addChild(node)
        WordNet.__extend_taxonomy_graph(node, synset_path[1:])

    
    def get_mln_similarity_and_sense_assertions(self, known_concepts, unknown_concepts):
        for i, unkwn in enumerate(unknown_concepts):
            for kwn in known_concepts:
                print '%.4f is_a(sense_%s, %s)' % (self.semilarity(unkwn, kwn), self.synset(unkwn).lemmas[0].name, kwn)
            print
    
    def asGraphML(self):
        '''
        Prints a GraphML string to the specified stream.
        '''
        if self.core_taxonomy is None:
            raise Exception('Need a collapsed taxonomy')
        tax = self.core_taxonomy
        g = Graph()
        processed = {}
        for c in tax.traverse():
            if c.data in processed: continue
            n_child = GMLNode(g, label=c.data, color='#dddddd', model='rgb')
            for p in c.parents:
                if not p.data in processed: 
                    n_parent = GMLNode(g, label=p.data)
                else:
                    n_parent = processed[p.data]
                Edge(g, n_child, n_parent)
                processed[p.data] = n_parent
            processed[c.data] = n_child
        return g
        

if __name__ == '__main__':
    known_concepts = ['milk.n.01', 'bowl.n.01', 'bowl.n.04', 'cup.n.01', ]
    unknown_concepts = ['mug.n.04', 'glass.n.02', 'tea.n.01', 'milk.n.01', 'cup.n.02']
    wn = WordNet(list(set(known_concepts + unknown_concepts)))
    wn.asGraphML(file('/home/nyga/tmp/wordnet.graphml', 'w+'))
    wn.get_mln_similarity_and_sense_assertions(known_concepts, unknown_concepts)
    
#     print 'semil: %.3f' % wn.semilarity(c1, c2)
#     print 'wup:   %.3f' % wn.wup_similarity(c1, c2)
