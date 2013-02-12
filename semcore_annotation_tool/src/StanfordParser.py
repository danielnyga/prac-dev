import jpype
import re
import platform

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
        
    def getDependencies(self, sentence=None, collapsed=False):
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
    
