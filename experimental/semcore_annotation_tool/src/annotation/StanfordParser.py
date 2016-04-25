import jpype
import re
import platform
from parsing import Trees
from prac_nltk.corpus import wordnet as wn

class ParserError(Exception):
    def __init__(self, *args, **margs):
        Exception.__init__(self, *args, **margs)
        
class Dependency(object):
    '''
    This class represents one single Stanford dependency.
    '''
        
    def __init__(self, depName, dep, gov):
        self.depName = depName
        self.dep = dep
        self.gov
        
    def __str__(self):
        return '%s(%s, %s)' % (self.depName, self.dep, self.gov)

class Parser(object):

    def __init__(self, pcfg_model_fname=None):
        self.pcfg_model_fname = pcfg_model_fname
        self.package_lexparser = jpype.JPackage("edu.stanford.nlp.parser.lexparser")
        self.package_trees = jpype.JPackage('edu.stanford.nlp.trees')
        self.package = jpype.JPackage("edu.stanford.nlp")
        self.parser = self.package_lexparser.LexicalizedParser(self.pcfg_model_fname, ['-retainTmpSubcategories', '-maxLength', '160'])
        
    def getPOSandDep(self, sentence=None, collapsed=False):
        result = []
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
        result.append(self.getDependencies(collapsed=collapsed))
        result.append(self.getPOS())
        return result
    
    #Returns the parse tree of a sentence
#    def getParseTree(self,sentence=None):
#        if sentence is not None:
#            self.parse = self.parser.apply(sentence)
#        tlp = self.package_trees.PennTreebankLanguagePack()
#        puncWordFilter = tlp.punctuationWordRejectFilter()
#        gsf = tlp.grammaticalStructureFactory(puncWordFilter)
#        gs = gsf.newGrammaticalStructure(self.parse)
#        #Build tree
#        t = dict()
#        while not c.parent() is None:
#            for c in gs.root().getLeaves():
#                t[str(c.parent)] = c.parent()
            
    
    def getDependencies(self, sentence=None, collapsed=True):
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
        tlp = self.package_trees.PennTreebankLanguagePack()
        puncWordFilter = tlp.punctuationWordRejectFilter()
        gsf = tlp.grammaticalStructureFactory(puncWordFilter)
        gs = gsf.newGrammaticalStructure(self.parse)
        if collapsed is True:
            deps =  gs.typedDependenciesCollapsed()
        else:
            deps = gs.typedDependencies()
            
        return deps
    
    def getSEs(self, sentenceID, sentence=None):
        possibleElements = ['ADJP','ADVP','CONJP','FRAG','INTJ','LST','NAC','NP','NX','PP','PRN','PRT','QP','RRC','UCP','VP','WHADJP','WHAVP','WHNP','WHPP','X']
        tmp = dict()
        tree = self.createParseTree(sentenceID,sentence)
        for pe in possibleElements:
            tmp[pe] = self.parseTree(tree, pe)
        result = dict()
        for r, value in tmp.iteritems():
            for t in value:
                result[r] = dict()
                nw = self.getLeafElements(t, True)
                result[r][str(t.element.identifier)]= nw
        return [result, self.getAllLeafs(tree), tree]
    
    def parseTree(self, tree, searchTerm, queryType='type'):
        """Traverse the Tree and Find the subtrees that have the Search term as root"""
        result = []
        if queryType == 'type':
            if str(tree.element.type) == searchTerm:
                result.append(tree)
                return result
        elif queryType == 'identifier':
            if str(tree.element.identifier) == searchTerm:
                result.append(tree)
                return result
        if tree.children is None:
            return []
        for child in tree.children:
            result = result + self.parseTree(child, searchTerm, queryType)
        return result
    
    def getLeafElements(self,tree, start):
        """get the words of all the leafs"""
        possibleElements = ['ADJP','ADVP','CONJP','FRAG','INTJ','LST','NAC','NP','NX','PP','PRN','PRT','QP','RRC','UCP','VP','WHADJP','WHAVP','WHNP','WHPP','X']
        res = []
        #if tree.element.type in possibleElements and not start:
        #    res.append(tree.element)
        #    return res
        start = False
        if tree.children is None or len(tree.children)==0:
            res.append(tree.element)
            return res
        for child in tree.children:
            res = res + self.getLeafElements(child, start)
        return res
                
    def _getAllLeafs(self, rootNode):
            res = []
            if len(rootNode.children) == 0:
                res.append(rootNode.element)
                return res
            for c in rootNode.children:
                res = res + self._getAllLeafs(c)
            return res
        
    def getAllLeafs(self, tree):
        return self._getAllLeafs(tree)
    
    def createParseTree(self, sentenceID, sentence=None):
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
        else:
            return None
        tlp = self.package_trees.PennTreebankLanguagePack()
        gsf = tlp.grammaticalStructureFactory()
        parse = self.parser.apply(sentence)
        gs = gsf.newGrammaticalStructure(parse)
        
        tree = self.getTree(gs, sentenceID)
        return tree
    
    def getTree(self, gs, sentenceID):
        tree = self._getTree(gs.root(),sentenceID)
        return tree
        
    def _getTree(self, node,sentenceID):
        res = Trees.SyntacticTree()
        wnToSPPOS = {'n':'NN','s':'JJ','v':'VB','a':'JJ','r':'RB'}
        res.element = Trees.SyntacticElement(node.label().word(), node.toString())
        res.element.type = node.label().value()
        if not res.element.name is None:
            res.element.wordnetPOSs = set([wnToSPPOS[x.pos] for x in wn.synsets(res.element.name)])
        if len(node.children())>0:
            for child in node.children():
                res.addChild(self._getTree(child, sentenceID), sentenceID)
        #else:
        
        
        return res
    
    def getPOS(self, sentence=None):
        if sentence is not None:
            self.parse = self.parser.apply(sentence)
        tokens = [str(x) for x in self.parse.taggedYield()]
        pos = dict()
        commaOffset = 0
        for i,t in enumerate(tokens):
            w,p = t.rsplit('/',1)
            if not re.match(r'[a-zA-z0-9]+', p): 
               # if p==",":
                   # commaOffset = commaOffset+1    
                continue
            pos[i+1]=(['%s-%d'% (w, i+1-commaOffset), p])
        return pos
        

    def printInfo(self):
        Numberer = self.package.util.Numberer
        print ("Grammar\t" + 
               `Numberer.getGlobalNumberer("states").total()` + '\t' + 
               `Numberer.getGlobalNumberer("tags").total()` + '\t' + 
               `Numberer.getGlobalNumberer("words").total()` + '\t' + 
               `self.parser.pparser.ug.numRules()` + '\t' + 
               `self.parser.pparser.bg.numRules()` + '\t' + 
               `self.parser.pparser.lex.numRules()`)

        print "ParserPack is ", self.parser.op.tlpParams.getClass()
        print "Lexicon is ", self.parser.pd.lex.getClass()        
        print "Tags are: ", Numberer.getGlobalNumberer("tags")
        self.parser.op.display()
        print "Test parameters"
        self.parser.op.tlpParams.display();
        self.package_lexparser.Test.display()
        
        
    def parse(self, sentence):
        """
        Parses the sentence string, returning the tokens, and the parse tree as a tuple.
        tokens, tree = parser.parse(sentence)
        """
        tokens = self.documentPreprocessor.getWordsFromString(sentence)
        for token in tokens:
            if token.word() in ["down"]:
                print "setting tag"
                token.setTag("IN")
                pass
            if token.word().lower() in ["bot"]:
                token.setTag("NN")
                pass
        
        wasParsed = self.parser.parse(tokens)
        if not wasParsed:
            raise ParserError("Could not parse " + sentence)
        return tokens, self.parser.getBestParse()
class Tree:
    def __init__(self,root):
        self.root = root
    
class TreeNode:
    def __init__(self, parent=None, children=None):
        if not parent is None:
            self.parent = parent
        else:
            self.parent = None
        if not children is None:
            self.children = children 
        else:
            self.children = []
    
#    def parseToStanfordDependencies(self, sentence):
#
#        tokens, tree = self.parse(sentence)
#        standoffTokens = [standoffFromToken(sentence, token)
#                          for token in tokens]
#        posTags = [token.tag() for token in tree.taggedYield()]
#        print " ".join(["%s/%s" % (word.text, tag) for word, tag in zip(standoffTokens, posTags)])
#        #print tree.taggedYield().toString(False)
#        result = self.package.trees.EnglishGrammaticalStructure(tree)
#        
#        returnList = []
#        for dependency in result.typedDependenciesCollapsedTree():
#
#            govStandoff = standoffTokens[dependency.gov().index() - 1]
#            depStandoff = standoffTokens[dependency.dep().index() - 1]
#
#            returnList.append((str(dependency.reln()),
#                               govStandoff,
#                               depStandoff))
#        return Dependencies(sentence, standoffTokens, posTags, returnList)
    
