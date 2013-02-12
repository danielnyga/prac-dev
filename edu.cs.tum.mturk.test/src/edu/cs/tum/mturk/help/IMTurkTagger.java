package edu.cs.tum.mturk.help;

import java.util.ArrayList;

import edu.cs.tum.mturk.model.MTurkWord;

public interface IMTurkTagger {

	//Tag sentence with POS tags
	public ArrayList<MTurkWord> tag(String sentence) throws Exception;
	
	//Split sentence into token
	public ArrayList<String> tokenize(String sentence) throws Exception;
}
