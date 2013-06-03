import os
import sys

sys.path.append('./lib/jyson-1.0.2.jar')
sys.path.append('../edu.cs.tum.mturk.test/bin')
sys.path.append('../edu.cs.tum.wordnet/lib/wordnet30.jar')
sys.path.append('../edu.cs.tum.wordnet/lib/edu.mit.jwi_2.1.5.jar')
sys.path.append('%s/stanford-parser.jar' % os.getenv('STANFORD_PARSER_HOME'))

from edu.cs.tum.mturk.pos.parser import Parser
from edu.cs.tum.mturk.pos.parser import SyntaxTree
from edu.tum.cs.ias.wordnet import WordNet3 as wn
from edu.tum.cs.ias.wordnet import MySynsetID
from edu.cs.tum.mturk.model import MTurkWord
from edu.cs.tum.mturk import CompoundNouns
from edu.mit.jwi.item import POS
from com.xhaus.jyson import JysonCodec as json
import re


def getWordWithIndex(index, words):
	for w in words:
		if w.get('index') == index:
			return w
	return None	

def getSenseKey(word, synsetid):
	id = MySynsetID.parseSynsetID(synsetid)
	keys = wn.getSenseKeys(id)
	w = wn.getWordStem(word, POS.getPartOfSpeech(synsetid[-1]))
	for k in keys:
		if k.getLemma().replace('_', ' ') == w:
			return k
	return None

if __name__ == '__main__':
	jsonfile = open('../data/live_2/STIR/Results.summarized')
	json_obj = json.loads(jsonfile.read())
	
	p = Parser('%s/grammar/englishPCFG.ser.gz' % os.getenv('STANFORD_PARSER_HOME'))
	
	f = open('../data/stirring.senses', 'w')
	
	count_sentences = 0
	
	for obj in json_obj:
		sentence = obj.get('context')
		words = obj.get('words')
		indices = [word.get('index') for word in words]
		indices.sort()
		cindices = [word.get('cindex') for word in words]
#		cwords = [words[i].get('word') for i in indices]
		print sentence
		
		sense_words = dict([(w.get('index'), w.get('word')) for w in words])
		sense_lemmata = dict([(w.get('index'), w.get('lemma')) for w in words])
		
		tokens = SyntaxTree.leaves(p.parse(sentence))
		
		for i, token in enumerate(tokens):
			tokens[i] = MTurkWord(token.getName(), token.getType(), i)
		
		comp_nouns = CompoundNouns.getWordNetInfos(tokens)
		
		for i, token in enumerate(tokens):
			if i in indices:
				synsetstr = getWordWithIndex(i, words).get('sense')
				line_str = str(count_sentences) + '\t' + str(i) + '\t' + sense_words[i] + '\t' + synsetstr + '\t' + str(getSenseKey(sense_lemmata[i], synsetstr)) + '\t'
				print line_str
				f.write(line_str.encode('utf-8') + '\n')
			elif i in cindices:
				continue
			else:
				line_str = str(count_sentences) + '\t' + str(i) + '\t' + token.getValue() + '\t' + '\t' + '\t'
				print line_str
				f.write(line_str.encode('utf-8') + '\n')
		print '================================================='
		count_sentences+=1
		
	f.close()
