from itertools import chain
import urllib2
import json
from pracmln.utils.graphml import Graph, Node as GMLNode, Edge
from prac.pracutils.graph import DAG , Node
import os
import graphviz as gv
import properties
from scipy import spatial
from pracmln.praclog import logger
from prac.pracutils.pracgraphviz import render_gv

log = logger(__name__)

WUP_SIM_LINK = "http://strazdas.vdu.lt:8081/AcatWSOntology4/rest/similarity/{}/{}"
SYNSET_LINK = "http://strazdas.vdu.lt:8081/AcatWSOntology4/rest/synsets/{}"
HYPERNYMS_LINK = "http://strazdas.vdu.lt:8081/AcatWSOntology4/rest/hypernyms/{}"

PRAC_HOME = os.environ['PRAC_HOME']
NLTK_POS = ['n', 'v', 'a', 'r']

colorsims = {}
shapesims = {}
sizesims = {}
consistencysims = {}
dimensionsims = {}

known_concepts = ['hydrochloric_acid.n.01',
                  'drop.n.02',
                  'sodium_hydroxide.n.01',
                  'water.n.06',
                  'liter.n.01',
                  'milliliter.n.01',
                  'morsel.n.01',
                  'test_tube.n.01',
                  'bottle.n.01',
                  'petri_dish.n.01',
                  'silver_nitrate.n.01'
                  #'soup.n.01', 
#                   'milk.n.01', 
#                   'water.n.06', 
#                   'cup.n.01', 
#                   'cup.n.02', 
#                   'glass.n.02', 
#                   'bowl.n.03', 
# #                   'coffee.n.01', 
#                   'bowl.n.04',
#                   'spoon.n.01',
#                   'spoon.n.02',
#                   'sauce.n.01',
#                   'salt.n.02',
#                   'pepper.n.03',
#                   'marjoram.n.01',
#                   'rosemary.n.01',
#                   'tomato_sauce.n.01',
#                   'carbonara.n.01',
#                 'batter.n.02',
#                   'baking_tray.n.01',
#                   'cheese.n.01',
#                   'mozzarella.n.01',
#                   'water_faucet.n.01',
#                 'oven.n.01',
#                   'degree_celsius.n.01',
#                   'refrigerator.n.01',
#                   'stove.n.01'
#                 'one.n.01',
#                 'two.n.01',
#                 'three.n.01',
#                 'four.n.01',
#                 'five.n.01',
#                 'six.n.01',
#                 'seven.n.01',
#                 'eight.n.01',
#                 'nine.n.01',
                  ]

class Synset():
    def __init__(self, name):
        if name is None :
            self.name = None
            self.pos = None
        else :
            self.name = str(name)
            self.pos = str(name).split(".")[1]
            
    def hypernyms(self):
        request_answer = urllib2.urlopen(HYPERNYMS_LINK.format(self.name)).read()
        result = []
        
        try:
            json_obj = json.loads(request_answer)
            paths = json_obj['paths']
            
            for element in paths:
                path = element['path']
                temp = []
                result.append(Synset(path[1]['synsetName']))
                
        except Exception as e:
            log.error("No hypernyms for " + self.name)
            return result
        
        return result
    
    def instance_hypernyms(self):
        #TODO implement 
        return []
    
    def hypernym_paths(self):
        request_answer = urllib2.urlopen(HYPERNYMS_LINK.format(self.name)).read()
        result = []
        
        try:
            json_obj = json.loads(request_answer)
            paths = json_obj['paths']
            
            for element in paths:
                path = element['path']
                temp = []
                for synset in path:
                    temp.append(Synset(synset['synsetName']))
                #To keep consistent with the nltk wrapper
                temp = list(reversed(temp))
                result.append(temp)
                
        except Exception as e:
            log.error(request_answer)
        
        return result
    
    def __repr__(self):
        return 'Synset(%r)' % (self.name)
    
    def _iter_hypernym_lists(self):
        """
        @return: An iterator over L{Synset}s that are either proper
        hypernyms or instance of hypernyms of the synset.
        """
        todo = [self]
        seen = set()
        while todo:
            for synset in todo:
                seen.add(synset)
            yield todo
            todo = [hypernym
                    for synset in todo
                    for hypernym in (synset.hypernyms() + \
                        synset.instance_hypernyms())
                    if hypernym not in seen]
            
    def max_depth(self):
        """
        @return: The length of the longest hypernym path from this
        synset to the root.
        """

        if "_max_depth" not in self.__dict__:
            hypernyms = self.hypernyms() + self.instance_hypernyms()
            if not hypernyms:
                self._max_depth = 0
            else:
                self._max_depth = 1 + max(h.max_depth() for h in hypernyms)
        return self._max_depth
            
    def min_depth(self):
        """
        @return: The length of the shortest hypernym path from this
        synset to the root.
        """

        if "_min_depth" not in self.__dict__:
            hypernyms = self.hypernyms() + self.instance_hypernyms()
            if not hypernyms:
                self._min_depth = 0
            else:
                self._min_depth = 1 + min(h.min_depth() for h in hypernyms)
        return self._min_depth


    def shortest_path_distance(self, other):
        """
        Returns the distance of the shortest path linking the two synsets (if
        one exists). For each synset, all the ancestor nodes and their
        distances are recorded and compared. The ancestor node common to both
        synsets that can be reached with the minimum number of traversals is
        used. If no ancestor nodes are common, None is returned. If a node is
        compared with itself 0 is returned.

        @type  other: L{Synset}
        @param other: The Synset to which the shortest path will be found.
        @return: The number of edges in the shortest path connecting the two
            nodes, or None if no path exists.
        """

        if self == other:
            return 0

        path_distance = None

        dist_list1 = self.hypernym_distances()
        dist_dict1 = {}

        dist_list2 = other.hypernym_distances()
        dist_dict2 = {}

        # Transform each distance list into a dictionary. In cases where
        # there are duplicate nodes in the list (due to there being multiple
        # paths to the root) the duplicate with the shortest distance from
        # the original node is entered.

        for (l, d) in [(dist_list1, dist_dict1), (dist_list2, dist_dict2)]:
            for (key, value) in l:
                if key in d:
                    if value < d[key]:
                        d[key] = value
                else:
                    d[key] = value

        # For each ancestor synset common to both subject synsets, find the
        # connecting path length. Return the shortest of these.

        for synset1 in dist_dict1.keys():
            for synset2 in dist_dict2.keys():
                if synset1 == synset2:
                    new_distance = dist_dict1[synset1] + dist_dict2[synset2]
                    if path_distance < 0 or new_distance < path_distance:
                        path_distance = new_distance

        return path_distance
        
    def hypernym_distances(self, distance=0):
        """
        Get the path(s) from this synset to the root, counting the distance
        of each node from the initial node on the way. A set of
        (synset, distance) tuples is returned.

        @type  distance: C{int}
        @param distance: the distance (number of edges) from this hypernym to
            the original hypernym L{Synset} on which this method was called.
        @return: A set of (L{Synset}, int) tuples where each L{Synset} is
           a hypernym of the first L{Synset}.
        """
        distances = set([(self, distance)])
        for hypernym in self.hypernyms() + self.instance_hypernyms():
            distances |= hypernym.hypernym_distances(distance+1)
        return distances
    
    def lowest_common_hypernyms(self, other):
        """Get the lowest synset that both synsets have as a hypernym."""

        self_hypernyms = self._iter_hypernym_lists()
        other_hypernyms = other._iter_hypernym_lists()

        synsets = set(s for synsets in self_hypernyms for s in synsets)
        others = set(s for synsets in other_hypernyms for s in synsets)
        synsets.intersection_update(others)

        try:
            max_depth = max(s.min_depth() for s in synsets)
            return [s for s in synsets if s.min_depth() == max_depth]
        except ValueError:
            return []

class WordNet(object):
    
    def __init__(self, concepts=known_concepts):
        self.core_taxonomy = None
        if concepts is not None:
            #self.initialize_taxonomy(concepts)
            self.initialize_csimilarities(colorsims, properties.chrcolorspecs, properties.achrcolorspecs)
            self.initialize_similarities(shapesims, properties.shapespecs)
            self.initialize_similarities(sizesims, properties.sizespecs)
            # self.initialize_similarities(consistencysims, properties.consistencyspecs)
            # self.initialize_similarities(dimensionsims, properties.dimensionspecs)


    def initialize_similarities(self, simdct, specs):
        # calculate euclidean distance between HSV values
        maxDist = 0.
        for k in specs.keys():
            simdct[k] = {}
            for c in specs.keys():
                simdct[k][c] = spatial.distance.euclidean(specs[k],specs[c])
                maxDist = max(maxDist,simdct[k][c])

        # normalize
        for x in simdct:
            for y in simdct:
                simdct[x][y] = 1-(simdct[x][y]/maxDist)

    def initialize_csimilarities(self, colorsims, specs, achrspecs):
        maxDist = 0.
        tempDict = dict(specs.items() + achrspecs.items())
        for k in tempDict.keys():
            colorsims[k] = {}
            for c in tempDict.keys():
                if k == c: # same color
                    colorsims[k][c] = 0.0
                elif not (k in specs and c in specs) and not (k in achrspecs and c in achrspecs): # one chromatic, one achromatic
                    colorsims[k][c] = 130.
                elif abs(tempDict[k][0] - tempDict[c][0]) > 180: # colors on different halves of the hue-circle
                    a = [(tempDict[k][0]+180)%360,tempDict[k][1],tempDict[k][2]]
                    b = [(tempDict[c][0]+180)%360,tempDict[c][1],tempDict[c][2]]
                    colorsims[k][c] = spatial.distance.euclidean(a,b)
                else:
                    colorsims[k][c] = spatial.distance.euclidean(tempDict[k],tempDict[c])
                maxDist = max(maxDist,colorsims[k][c])

        # normalize
        for x in colorsims:
            for y in colorsims:
                temp = 1-(colorsims[x][y]/maxDist)
                colorsims[x][y] = temp

            
    def initialize_taxonomy(self, concepts=None, collapse=True):
        '''
        Creates a new taxonomy given a set of concepts. If collapse is True,
        all subpaths with only one child and parent are collapsed. 
        '''
        entity_name = 'entity.n.01'
        self.core_taxonomy = DAG(root=Node(entity_name, entity_name))
        self.known_concepts = {entity_name: self.core_taxonomy.root}
        if concepts is None:
            self.core_taxonomy = None
        else:
            for direction in ('down', 'up'):
                for concept in concepts:
                    if concept == 'null': continue
                    synset = Synset(concept)
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
        if not pos in NLTK_POS:
            log.error('Unknown POS tag: %s' % pos)
            return []
        
        request_answer = urllib2.urlopen(SYNSET_LINK.format(word)).read()
        synsets = []
        
        try:
            json_obj = json.loads(request_answer)
            data = json_obj['data']
            
            for element in data:
                synsets.append(Synset(element["synset"]))
        except Exception as e:
            log.error(request_answer)
        
        return synsets
    
    def synset(self, synset_id):
        '''
        Returns the NLTK synset for the given id.
        '''
        if synset_id == 'null':
            return None
        try: 
            synset = Synset(synset_id)
            return synset
        except Exception, e:
            log.error('Could not obtain synset with ID "%s"' % synset_id)
            raise e
        
        
    def wup_similarity(self, synset1, synset2):
        request_answer = urllib2.urlopen(WUP_SIM_LINK.format(synset1, synset2)).read()
        sim = 0.0
        
        try:
            json_obj = json.loads(request_answer)
            sim = float(json_obj['SimilarityValue'])
        
        except Exception as e:
            log.error(request_answer)
        
        return sim


    def path_similarity(self, synset1, synset2):
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
        #
        # similarity = synset1.path_similarity(synset2)
        # not implemented, therefore
        similarity = self.wup_similarity(synset1.name, synset2.name)

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
            synsets.intersection_update(map(lambda syn: Synset(syn), self.known_concepts))
            others.intersection_update(map(lambda syn: Synset(syn), self.known_concepts))
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
                iterable = chain(data, iterable)
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
        posDiff = 0.

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

        # separate check for shape similarity
        if synset1.name in shapesims and synset2.name in shapesims:
            return shapesims[synset1.name][synset2.name]
        elif synset1.name in shapesims or synset2.name in shapesims: # shapes are maximially dissimilar to everything else
            return 0.

        # separate check for size similarity
        if synset1.name in sizesims and synset2.name in sizesims:
            return sizesims[synset1.name][synset2.name]
        elif synset1.name in sizesims or synset2.name in sizesims: # sizes are maximially dissimilar to everything else
            return 0.    

        # separate check for consistency similarity
        if synset1.name in consistencysims and synset2.name in consistencysims:
            return consistencysims[synset1.name][synset2.name]
        elif synset1.name in consistencysims or synset2.name in consistencysims: # consistencies are maximially dissimilar to everything else
            return 0.    

        # separate check for dimension similarity
        if synset1.name in dimensionsims and synset2.name in dimensionsims:
            return dimensionsims[synset1.name][synset2.name]
        elif synset1.name in dimensionsims or synset2.name in dimensionsims: # dimensions are maximially dissimilar to everything else
            return 0.  

        
        if synset1.pos in ADJ_POS:
            syns1 = self.unpackNoun(synset1)
            if len(syns1) == 0: return 0.
            synset1 = syns1[0]
            posDiff += .5
        if synset2.pos in ADJ_POS:
            syns2 = self.unpackNoun(synset2)
            if len(syns2) == 0: return 0.
            synset2 = syns2[0]
            posDiff += .5
        if synset1 is None or synset2 is None:
            return 0.
        if synset1 == synset2:
            return 1.0              

        # additional knowledge: if one synset is in the hypernym path of the other, they are considered
        # closely related
        if self.synsHypRelation(synset1, synset2) > 0.: #return self.synsHypRelation(synset1, synset2)
            return self.synsHypRelation(synset1,synset2)
            # return self.synsHypRelation(synset1,synset2)

        # add additional knowledge: decrease similarity of synsets from different taxonomy branches
        # if not synsInPreDefTaxonomyBranch(synset1, synset2): posDiff += .5
        posDiff += 1 - self.synsTaxonomyBranchRelation(synset1, synset2)
        return self.wup(synset1,synset2,posDiff)

    def wup(self, synset1, synset2, posDiff=0.):
        # equates WUP Similarity: 2 * depth(lowestCommonHypernym) / depth(synset1) + depth(synset2) because:
        # depth(X) == X.max_depth() + 1
        # posDiff is used to decrease the similarity if one of the synsets is an adjective, to punish
        # inferring another synset which is used for similarity check
        lcss = synset1.lowest_common_hypernyms(synset2)
        if len(lcss) == 0: return 0.
        lcs = lcss[0]
        dlcs = lcs.max_depth() + 1
        ds1 = synset1.shortest_path_distance(lcs)
        ds2 = synset2.shortest_path_distance(lcs)
        if ds1 == None or ds2 == None: return 0.
        ds1 += dlcs
        ds2 += dlcs

        return 2. * dlcs / (ds1 + ds2 + posDiff)


    def synsTaxonomyBranchRelation(self, synset1, synset2):
        if not synset1.lowest_common_hypernyms(synset2): return 0
        return min([x.min_depth() for x in synset1.lowest_common_hypernyms(synset2)]) / max(synset1.min_depth(), synset2.min_depth())

    def synsHypRelation(self, syn1, syn2):
        lch = syn1.lowest_common_hypernyms(syn2)    
        if not syn1 in lch and not syn2 in lch: return 0.
        if not any(syn2 in path for path in syn1.hypernym_paths()):
            syn1Len = min([float(len(x)) for x in syn1.hypernym_paths()])
        else:
            for path in syn1.hypernym_paths():
                if not syn2 in path: continue
                else:
                    syn1Len = float(len(path))
        if not any(syn1 in path for path in syn2.hypernym_paths()):
            syn2Len = min([float(len(x)) for x in syn2.hypernym_paths()])
        else:
            for path in syn2.hypernym_paths():
                if not syn1 in path: continue
                else:
                    syn2Len = float(len(path))

        return 1. - (abs(syn1Len - syn2Len) / max(syn1Len, syn2Len))


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
        if type(synset) is str:
            synset = Synset(synset)
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
                log.info('%.4f is_a(sense_%s, %s)' % (self.semilarity(unkwn, kwn), self.synset(unkwn).lemmas[0].name, kwn))

    def asGraphML(self):
        '''
        Prints a GraphML string to the specified stream.
        '''
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
    
    
    def to_dot(self):
        if self.core_taxonomy is None:
            raise Exception('Need a collapsed taxonomy')
        tax = self.core_taxonomy
        g = gv.Digraph(format='svg')
        g.attr('graph', nodesep='.5', splines='true', rankdir='BT', ratio='fill', bgcolor='white')
        processed = {}
        for c in tax.traverse(algo='BFS'):
            g.node(c.data, fillcolor='#dddddd', style='filled', shape='box', fontname='Helvetica, Arial, sans-serif', size='2,2')
            processed[c.data] = c.data
        for p in tax.traverse(algo='BFS'):
            p_node = processed[p.data]
            for c in p.children:
                c_node = processed[c.data]
                g.edge(c_node, p_node, weight='1')
        return g
    
    def to_svg(self):
        g  = self.to_dot()
        return render_gv(g, 'wordnet.svg')
        
        

if __name__ == '__main__':

    wn = WordNet()
    print wn.to_svg()
            