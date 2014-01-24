'''
Created on Aug 31, 2012

@author: meyer
'''
from PyQt4 import QtCore, QtGui, uic, Qt
import utils 
import copy
import annotation.utilities as ut

class SyntacticTree:
    """A basic Syntactic Tree"""
    def __init__(self):
        self.element = None
        self.parent = None
        self.children = []
        self.sentence = None
        self.expression = ' '.join([x for x in self.inOrderLeafs()])
        
        self._tree_depth = None
        self._curr_depth = None
        
    def addChild(self, child,sentenceID):
        child.parent = self
        child.element.pos = copy.copy(self.element.type)
        child.sentence = sentenceID
        child.element.sentenceID = sentenceID
        self.children.append(child)
        
    def getSiblings(self):
        if self.parent is None:
            tmp = []
            tmp.append(self)
            return tmp
        else:
            return self.parent.children
    
    def getSubTreeByIdentifier(self, name):
        res = self._getSubTreeByIdentifier(self, str(name))
        return res
    
    def _getSubTreeByIdentifier(self, tree, name):
        res = None
        if tree.element.identifier == name:
            return tree
        
        if len(tree.children) > 0:
            for child in tree.children:
                tmp = self._getSubTreeByIdentifier(child, name)
                if tmp is not None:
                    res = tmp
        return res
    
    def inOrderNPs(self):
        for node in self._inOrderSearchTerms(self, 'NP'):
            yield node
    
    def inOrderPhrases(self, leafs = False):
        for node in self._inOrderSearchTerms(self, None):
            yield node         
    
    def _inOrderSearchTerms(self, tree, searchTerm):
        leaves = []
        
        if tree.element.type==searchTerm or (
            searchTerm is None and not len(tree.children) ==0):
            leaves.append(tree)
        
        if len(tree.children) > 0:
            for child in tree.children:
                leaves += self._inOrderSearchTerms(child,searchTerm)
        
        return leaves
        
    def inOrderLeafs(self):
        """Generator to get the leaves inOrder."""
        for leaf in self._inOrderLeafs(self):
            yield leaf
            
    def _inOrderLeafs(self, tree):
        leaves = []
        
        if len(tree.children) > 0:
            for child in tree.children:
                leaves += self._inOrderLeafs(child)
        else:
            if tree.element is not None and not tree.element.name == '':
                leaves.append(tree)
        
        return leaves    
    
    def get_root_element(self):
        """Return the root node of the tree."""
        
        tmp_node = self
        while True:
            if tmp_node.parent is None:
                break
            tmp_node = self.parent
        return tmp_node
    
    def syntacticElements(self, cat):
        """
        Return all leafs that have any of the POS that are specified in cat.
        
        """
        
        res = []
        for leaf in self.inOrderLeafs():
            if leaf.element.pos in cat:
                res.append(leaf)
        return res
    
    def possibleElements(self, cat):
        """
        Return all leafs that match any of the POS given in 
        cat in their possible WN POS.
        
        """
        
        inlist = []
        for y in self.inOrderLeafs():
            map(lambda x : inlist.append(y) if x in cat else None, 
                y.element.wordnetPOSs)
        return inlist
        
        
    def printTree(self):
        identation = 2
        res = '-' * identation
        print '{identat}{ident}\n'.format(identat = res, 
                                          ident = self.element.identifier)
        if len(self.children) == 0: return
        for child in self.children:
            child.printTree()
            
    def _getdepth(self, node):
        if node is not None:
            self._curr_depth += 1
            if(self._curr_depth > self._tree_depth):
                self._tree_depth = self._curr_depth
            for i in node.children:
                self._getdepth(i)
            self._curr_depth -= 1
            
    def getdepth(self):
        self._tree_depth = 0
        self._getdepth(self)
        
        return self._tree_depth
        
    def leaves(self, tree):
        leaves = []
        
        if len(tree.children) > 0:
            for i in tree.children:
                t = i
                leaves.append(self.leaves(t))
        else:
            if tree.element is not None and not tree.element.name == '':
                leaves.append(tree.element)
        
        return leaves
    
    def getQtElement(self):
        res = QtGui.QTreeWidgetItem([str(self.element.identifier)],0)
        if not len(self.children) == 0:
            for child in self.children:
                res.addChild(child.getQtElement())
        return res

class SyntacticElement:
    """The Leave Nodes of a Syntactic Tree"""
    def __init__(self, name, identifier):
        self.name = name
        self.identifier = identifier
        self.pos = None
        self.sentenceID = ''
        self.groundIdentifier = None
        self.wordnetPOSs = []
        self._type = None
        
    @property
    def mlngroundidentifier(self):
        res = ut.helper.handleSpecialChars(self.groundIdentifier.strip("'"))
        res = utils.toMLNCompatibleFormat(res)
        return res
        
    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, val):
        self._type = str(val)
        
    @property
    def mlnidentifier(self):
        ident = utils.toMLNCompatibleFormat(self.identifier)
        res = '{identifier}_S_{sentence}'.format(identifier = ident, 
                                                 sentence = self.sentenceID)
        res = ut.helper.handleSpecialChars(res)
        return res
        
class TreeGenerator:
    """Creates Trees"""
    def __init__(self):
        pass
    
    def generateSyntactTreeFromParseTreeString(self, treeString):
        """ Create a SyntacticTree fromt the ouput of the Stanford Parser"""
        
        pass
    
    def tokenize(self,treeString):
        stack = 0
        start = -1
        res = []
        
        for i, value in enumerate(treeString):
            if value =='(':
                if stack == 0:
                    start = i+1 
                stack =+ 1
            elif value == ')':
                stack -= 1
                if stack ==0:
                    token = treeString[start:i]
                    res.append(token)
        return res
                
    
    
if __name__=='__main__':
    treeString = """(ROOT-6{CategoryAnnotation=ROOT, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, DirectObjectGRAnnotation=[NP-10], HeadTagAnnotation=VB-9, HeadWordAnnotation=Make-1, PartOfSpeechAnnotation=null, PredicateGRAnnotation=[VP-8], PunctuationGRAnnotation=[.-14], TextAnnotation=null} 
  (S-7{CategoryAnnotation=S, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=VB-9, HeadWordAnnotation=Make-1, PartOfSpeechAnnotation=null, TextAnnotation=null} 
    (VP-8{CategoryAnnotation=VP, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=VB-9, HeadWordAnnotation=Make-1, PartOfSpeechAnnotation=null, TextAnnotation=null} 
      (VB-9{CategoryAnnotation=VB, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=VB-9, HeadWordAnnotation=Make-1, PartOfSpeechAnnotation=null, TextAnnotation=null} 
        Make-1{CharacterOffsetBeginAnnotation=0, CharacterOffsetEndAnnotation=4, HeadWordAnnotation=Make-1, PartOfSpeechAnnotation=VB, TextAnnotation=Make}) 
      (NP-10{CategoryAnnotation=NP, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=NNP-13, HeadWordAnnotation=Brownies-4, NounCompoundModifierGRAnnotation=[NNP-11, NNP-12], PartOfSpeechAnnotation=null, TextAnnotation=null} 
        (NNP-11{CategoryAnnotation=NNP, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=NNP-11, HeadWordAnnotation=Chocolate-2, PartOfSpeechAnnotation=null, TextAnnotation=null} 
          Chocolate-2{CharacterOffsetBeginAnnotation=5, CharacterOffsetEndAnnotation=14, GovernorGRAnnotation=[Brownies-4], HeadWordAnnotation=Chocolate-2, PartOfSpeechAnnotation=NNP, TextAnnotation=Chocolate}) 
        (NNP-12{CategoryAnnotation=NNP, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=NNP-12, HeadWordAnnotation=Cherry-3, PartOfSpeechAnnotation=null, TextAnnotation=null} 
          Cherry-3{CharacterOffsetBeginAnnotation=15, CharacterOffsetEndAnnotation=21, GovernorGRAnnotation=[Brownies-4], HeadWordAnnotation=Cherry-3, PartOfSpeechAnnotation=NNP, TextAnnotation=Cherry}) 
        (NNP-13{CategoryAnnotation=NNP, CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=NNP-13, HeadWordAnnotation=Brownies-4, PartOfSpeechAnnotation=null, TextAnnotation=null} 
          Brownies-4{CharacterOffsetBeginAnnotation=22, CharacterOffsetEndAnnotation=30, GovernorGRAnnotation=[Make-1], HeadWordAnnotation=Brownies-4, PartOfSpeechAnnotation=NNP, TextAnnotation=Brownies}))) 
    (.-14{CategoryAnnotation=., CharacterOffsetBeginAnnotation=-1, CharacterOffsetEndAnnotation=-1, HeadTagAnnotation=.-14, HeadWordAnnotation=.-5, PartOfSpeechAnnotation=null, TextAnnotation=null} 
      .-5{CharacterOffsetBeginAnnotation=30, CharacterOffsetEndAnnotation=31, HeadWordAnnotation=.-5, PartOfSpeechAnnotation=., TextAnnotation=.})))"""
    import re
    treeString.split('(')
    tg = TreeGenerator()
    tokens = tg.tokenize(treeString)
    print tokens
    
    
    
    