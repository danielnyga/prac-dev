package edu.cs.tum.mturk.pos;

import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.List;

import edu.cs.tum.mturk.help.IMTurkTagger;
import edu.cs.tum.mturk.model.MTurkWord;
import edu.stanford.nlp.ling.HasWord;
import edu.stanford.nlp.ling.TaggedWord;
import edu.stanford.nlp.tagger.maxent.MaxentTagger;

public class StanfordPOSTagger implements IMTurkTagger{

	private MaxentTagger tagger;
	private boolean initialized = false;
	
	/**
	 * Initializes the POS tagger
	 * @throws IOException
	 * @throws ClassNotFoundException
	 */
	public void initialize() throws IOException, ClassNotFoundException
	{
		// Load tagger model
		tagger = new MaxentTagger("lib/left3words-wsj-0-18.tagger");
		
		initialized = true;
	}
	
	/**
	 * Tags a given sentence with the Stanford POS Tagger
	 * 
	 * @param sentenceToTag this sentence should be tokinized and tagged
	 * 
	 * @return the tagged sentence (list of tagged words)
	 * 
	 * @throws IOException
	 * @throws ClassNotFoundException
	 */
	public ArrayList<MTurkWord> tag(String sentenceToTag) throws IOException, ClassNotFoundException 
	{
		if(!initialized)
			this.initialize();
		
		//Tokenize the text
		List<List<HasWord>> sentences = MaxentTagger
				.tokenizeText(new StringReader(sentenceToTag));

		ArrayList<MTurkWord> taggedWords = new ArrayList<MTurkWord>();
		
		//For each sentences in the tokinized text (SHOULD BE 1)
		for (List<HasWord> sentence : sentences) 
		{
			//Tag the sentence
			ArrayList<TaggedWord> taggedSentence = tagger.tagSentence(sentence);
			
			int index = 0;
			for(TaggedWord word : taggedSentence)
			{
				MTurkWord tword = new MTurkWord(word.value(), word.tag(), index);
				taggedWords.add(tword);
				index++;
			}
		}

		return taggedWords;
	}

	@Override
	public ArrayList<String> tokenize(String sentence) throws Exception 
	{
		if(!initialized)
			this.initialize();
		
		List<List<HasWord>> sentences = MaxentTagger
		.tokenizeText(new StringReader(sentence));
		
		ArrayList<String> tokens = new ArrayList<String>();
		
		for(HasWord w : sentences.get(0))
		{
			tokens.add(w.word());
		}
		
		return tokens;
	}
}
