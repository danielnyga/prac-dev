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
nltk.data.path = [os.path.join('data', 'nltk_data')]

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
        return wordnet.synsets(word, pos)


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


if __name__ == '__main__':
    wn = WordNet(['fork.n.01', 'spoon.n.01'])
    for p in  wn.hypernym_paths('toaster.n.01'):
        print p