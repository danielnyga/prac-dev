package edu.cs.tum.mturk.pos;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.cs.tum.mturk.help.IMTurkTagger;
import edu.cs.tum.mturk.model.MTurkWord;
import edu.cs.tum.mturk.pos.parser.Parser;
import edu.cs.tum.mturk.pos.parser.SyntaxElement;
import edu.cs.tum.mturk.pos.parser.SyntaxTree;

public class StanfordPOSParser implements IMTurkTagger{

	private Parser parser;
	private boolean initialized;
	
	public void initialize() throws IOException, ClassNotFoundException
	{
		// Load tagger model
		parser = new Parser("lib/englishPCFG.ser.gz");
		
		initialized = true;
	}
	
	private SyntaxTree parseLocal(String sentence)
	{
		return parser.parseWithConsistencyCheck(sentence, 10000);
	}
	
	@Override
	public ArrayList<MTurkWord> tag(String sentence) throws Exception 
	{
		//init parser?
		if(!initialized)
			this.initialize();
		
		SyntaxTree tree = parseLocal(sentence);
		
		List<SyntaxElement> leaves = SyntaxTree.leaves(tree);
		
		//Collect all words
		ArrayList<MTurkWord> words = new ArrayList<MTurkWord>();
		int index = 0;
		
		//Convert words to MTurkWord repres.
		for(SyntaxElement elem : leaves)
		{
			MTurkWord word = new MTurkWord(elem.getName(), elem.getType(), index);
			words.add(word);
			
			index++;
		}
		
		return words;
	}

	@Override
	public ArrayList<String> tokenize(String sentence) throws Exception{
		if(!initialized)
			this.initialize();
		
		SyntaxTree tree = parseLocal(sentence);
		
		//Collect words
		List<SyntaxElement> leafs = SyntaxTree.leaves(tree);
		
		ArrayList<String> words = new ArrayList<String>();
		int index = 0;
		for(SyntaxElement elem : leafs)
		{
			words.add(elem.getName());
			
			index++;
		}
		
		return words;
	}

}
