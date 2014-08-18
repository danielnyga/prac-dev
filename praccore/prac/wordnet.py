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

from nltk.corpus import wordnet
from pracutils.graph import DAG , Node
import nltk
import logging 
from itertools import chain
from nltk.corpus.reader.wordnet import Synset
from utils.graphml import Graph, Node as GMLNode, Edge
import itertools
import os
from scipy import spatial

PRAC_HOME = os.environ['PRAC_HOME']
nltk.data.path = [os.path.join(PRAC_HOME, 'data', 'nltk_data')]

NLTK_POS = ['n', 'v', 'a', 'r']
TAXONOMY_BRANCHES = ['color.n.01','size.n.01','shape.n.01','kitchenware.n.01','tableware.n.01','container.n.01','plant_part.n.01','food.n.01']
colorsims = {}
# hsv values for color similarity calculation
colorspecs = {  'pink.s.01': (335,87,87),
                'purple.s.01': (290,87,87),
                'blue.s.01': (235,87,87),
                'cyan.s.01': (150,87,87),
                'light-blue.s.01': (175,87,87),
                'green.s.01': (115,87,87),
                'yellow.s.01': (50,87,87),
                'orange.s.01': (20,87,87),
                'brown.s.01': (20,87,97),
                'red.s.01': (0,87,87),
                'blood-red.s.01': (0,89,55),
                'black.a.01': (500,55,5), 
                'blackish.s.01': (500,55,5), 
                'white.a.01': (500,5,95),
                'whitish.s.02': (500,5,95),
                'grey.s.01': (500,5,50),
                'greyish.s.01': (500,5,50),
                'gray.s.01': (500,5,50),
                'grayish.s.01': (500,5,50)
                }

known_concepts = ['soup.n.01', 
                  'milk.n.01', 
                  'water.n.06', 
                  'cup.n.01', 
                  'cup.n.02', 
                  'glass.n.02', 
                  'bowl.n.03', 
                  'coffee.n.01', 
                  'bowl.n.04',
                  'spoon.n.01',
                  'spoon.n.02',
                  'one.n.01',
                  'two.n.01',
                  'three.n.01',
                  'four.n.01',
                  'five.n.01',
                  'six.n.01',
                  'seven.n.01',
                  'eight.n.01',
                  'nine.n.01',
                  ]


class WordNet(object):
    '''
    Wrapper class for WordNet, which may be initialized with
    some WordNet concepts spanning an initial, collapsed
    concept taxonomy.
    '''
    
    
    def __init__(self, concepts=known_concepts):
        self.core_taxonomy = None
        if concepts is not None:
            self.initialize_taxonomy(concepts)
            self.initialize_colorsimilarities()


    def initialize_colorsimilarities(self):
        # calculate euclidean distance between HSV values
        maxDist = 0.
        for k in colorspecs.keys():
            colorsims[k] = {}
            for c in colorspecs.keys():
                colorsims[k][c] = spatial.distance.euclidean(colorspecs[k],colorspecs[c])
                maxDist = max(maxDist,colorsims[k][c])

        # normalize
        for x in colorsims:
            for y in colorsims:
                colorsims[x][y] = 1-(colorsims[x][y]/maxDist)


            
    def initialize_taxonomy(self, concepts=None, collapse=True):
        '''
        Creates a new taxonomy given a set of concepts. If collapse is True,
        all subpaths with only one child and parent are collapsed. 
        '''
        log = logging.getLogger('WordNet')
        entity_name = 'entity.n.01'
        self.core_taxonomy = DAG(root=Node(entity_name, entity_name))
        self.known_concepts = {entity_name: self.core_taxonomy.root}
        if concepts is None:
            self.core_taxonomy = None
        else:
            for direction in ('down', 'up'):
                for concept in concepts:
                    if concept == 'null': continue
                    synset = wordnet.synset(concept)
                    paths = synset.hypernym_paths()
                    for path in paths:
                        if not path[0].name in self.known_concepts:
                            if direction == 'up':
                                self.known_concepts[path[-1].name] = Node(path[-1].name, path[-1].name)
                            continue
                        if direction == 'up':
                            path = list(reversed(path))
                        first = self.known_concepts[path[0].name]
                        self.__extend_taxonomy_graph(first, path[1:], direction=direction)
            # collapse the taxonomy
            if not collapse: return
            queue = list(self.core_taxonomy.root.children)
            while len(queue) > 0:
                node = queue.pop()
                if collapse:
                    # we keep the current node if it is a leaf or a fork (zero or more than one children)
                    keepThisNode = not (len(node.children) == 1 and len(node.parents) == 1)
                    for c in node.children:
                        keepThisNode |= len(c.parents) > 1
                    if not keepThisNode:
                        del self.known_concepts[node.data]
                        for p in node.parents:
                            p.children.remove(node)
                            p.children.update(node.children)
                        for c in node.children:
                            c.parents.remove(node)
                            c.parents.update(node.parents)
                queue.extend(node.children)
    
    def __extend_taxonomy_graph(self, concept, synset_path, direction):
        '''
        Takes a node of the taxonomy graph and a hypernymy path
        of a concept and extends the graph with the given path.
        - concept:     (Node) concept node to be extended (the root in most cases)
        - synset_path: a path as it is return by Synset.hypernym_paths(), for instance.
        '''
        log = logging.getLogger(__name__)
        if len(synset_path) == 0:
            return
        synset = synset_path[0]
        if synset.name not in self.known_concepts:
            node = Node(synset.name, synset.name)
            self.known_concepts[synset.name] = node
        else:
            node = self.known_concepts[synset.name]
        if direction == 'down':
            concept.addChild(node)
        elif direction == 'up':
            concept.addParent(node)
        else:
            raise Exception('Unkown direction: %s' % direction)
        self.__extend_taxonomy_graph(node, synset_path[1:], direction)


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
#         logging.getLogger().warning(synsets)
        return synsets


    def synset(self, synset_id):
        '''
        Returns the NLTK synset for the given id.
        '''
        if synset_id == 'null':
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
        return max(0.001, 0. if similarity is None else similarity)
    
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
    

    # returns generator of 1-dimensional list
    def flatten(self, iterable):
        iterable = iter(iterable)

        while 1:
            try:
                item = iterable.next()
            except StopIteration:
                break

            try:
                data = iter(item)
                iterable = itertools.chain(data, iterable)
            except:
                yield item

    # gets the synsets of the derivationally related forms of adjSynsets' Lemmas
    # flattens the list to get one-dimensional list without duplicates as result
    def unpackNoun(self, adjSynset):
        return list(set(self.flatten([drf.synset for drf in self.flatten([lemma.derivationally_related_forms() for lemma in adjSynset.lemmas])])))


    def similarity(self, synset1, synset2):
        '''
        Returns a custom semantic similarity for adjectives

        Note: the original wordnet similarity between an adjective and another
        object is always None. To be able to supply information
        about the similarity of adjectives, the derivationally 
        related forms (= nltk.corpus.reader.wordnet.Lemma)
        of the respecting Lemmas are retrieved. 
        The synsets of the resulting list of Lemmas have generally
        NLTK_POS 'n', and can therefore be used for a modified WUP 
        similarity, which adds a penalizing factor for inferring 
        another synset from adjectives.
        '''


        ADJ_POS = ['s','a']
        if type(synset1) is str:
            synset1 = self.synset(synset1)
        if type(synset2) is str:
            synset2 = self.synset(synset2)
        if synset1 is None or synset2 is None:
            return 0.
        if synset1 == synset2:
            return 1.0

        # separate check for color similarity
        if synset1.name in colorsims and synset2.name in colorsims:
            return colorsims[synset1.name][synset2.name]
        elif synset1.name in colorsims or synset2.name in colorsims: # colors are maximially dissimilar to everything else
            return 0.

        syns1 = [synset1]
        syns2 = [synset2]

        posDiff = 0.
        if synset1.pos in ADJ_POS:
            syns1 = self.unpackNoun(synset1)
            posDiff += .5
        if synset2.pos in ADJ_POS:
            syns2 = self.unpackNoun(synset2)
            posDiff += .5


        similarity = 0.
        for s1 in syns1:
            for s2 in syns2:
                # add additional knowledge: colors are dissimilar to shapes or sizes and vice versa:
                # decrease similarity of synsets from different taxonomy branches
                if not self.synsInSameTaxonomyBranch(s1, s2): posDiff += .5
                # equates WUP Similarity: 2 * depth(lowestCommonHypernym) / depth(s1) + depth(s2) because:
                # depth(X) == X.max_depth() + 1
                # posDiff is used to decrease the similarity if one of the synsets is an adjective, to punish
                # inferring another synset which is used for similarity check
                lcs = s1.lowest_common_hypernyms(s2)
                if lcs:
                    dlcs = lcs[0].max_depth() + 1
                    ds1 = s1.max_depth() + 1
                    ds2 = s2.max_depth() + 1

                    adjWupSim = 2 * dlcs / (ds1 + ds2 + posDiff)
                    similarity = max(similarity, adjWupSim)
        return 0. if similarity is None else similarity


    def synsInSameTaxonomyBranch(self, synset1, synset2):
        return any(tb in [hyp.name for hyp in self.flatten(synset1.hypernym_paths())] and tb in [hyp.name for hyp in self.flatten(synset2.hypernym_paths())] for tb in TAXONOMY_BRANCHES)            

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
        s = self.lowest_common_hypernyms(synset1, synset2, use_min_depth=True, simulate_root=True)
        if len(s) == 0:
            return 0.
        else:
            s = s[0]
        if s.name is None: return 0.
        h_s = self.get_subtree_height(s)
        return (h_r - h_s) / (h_r - .5 * (h_a + h_b))
    
    def get_subtree_height(self, synset):
        '''
        Returns the height of the subtree of the given synset.
        '''
        if type(synset) is str:
            synset = self.synset(synset)
        if synset is None:
            return 0
        assert type(synset) == Synset
#         if synset.name in self.subtree_cache:
#             return self.subtree_cache[synset.name]
        
        height = self.__get_subtree_height(synset)
#             self.subtree_cache[synset.name] = height
#             f = open(self.subtree_cache_path, 'w+')
#             pickle.dump(self.subtree_cache, f)
#             f.close()
        return height
        
        
    def __get_subtree_height(self, synset, current_height=0):
        hypos = synset.hyponyms()
        if self.core_taxonomy is not None:
            hypos = set(map(lambda s: s.name, hypos)).intersection(self.known_concepts)
            hypos = [self.synset(s) for s in hypos]
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
    
    
    
    def get_mln_similarity_and_sense_assertions(self, known_concepts, unknown_concepts):
        for i, unkwn in enumerate(unknown_concepts):
            for kwn in known_concepts:
                print '%.4f is_a(sense_%s, %s)' % (self.semilarity(unkwn, kwn), self.synset(unkwn).lemmas[0].name, kwn)
            print
    
    def asGraphML(self):
        '''
        Prints a GraphML string to the specified stream.
        '''
        log = logging.getLogger(self.__class__.__name__)
        if self.core_taxonomy is None:
            raise Exception('Need a collapsed taxonomy')
        tax = self.core_taxonomy
        g = Graph()
        processed = {}
        for c in tax.traverse(algo='BFS'):
            node = GMLNode(g, label=c.data, color='#dddddd', model='rgb')
#             log.info('adding node: %s' % node)
            processed[c.data] = node
        for p in tax.traverse(algo='BFS'):
            p_node = processed[p.data]
            for c in p.children:
                c_node = processed[c.data]
                Edge(g, c_node, p_node)
#                 log.info('adding edge %s ---> %s' % (c_node, p_node))
        return g
        

if __name__ == '__main__':
#     known_concepts = ['milk.n.01', 'bowl.n.01', 'bowl.n.04', 'cup.n.01', ]
#     unknown_concepts = ['mug.n.04', 'glass.n.02', 'tea.n.01', 'milk.n.01', 'cup.n.02']
    logging.getLogger().setLevel(logging.INFO)
    wn = WordNet()
    wn.asGraphML().write(file('/home/nyga/tmp/wordnet.graphml', 'w+'))
#     wn.get_mln_similarity_and_sense_assertions(known_concepts, unknown_concepts)
    
#     print 'semil: %.3f' % wn.semilarity(c1, c2)
#     print 'wup:   %.3f' % wn.wup_similarity(c1, c2)
