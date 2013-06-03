package edu.cs.tum.mturk;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.cs.tum.mturk.model.MTurkWord;
import edu.mit.jwi.item.ISynset;
import edu.mit.jwi.item.POS;
import edu.tum.cs.ias.wordnet.WordNet3;

public class CompoundNouns {
	/**
	 * Collect synsets for each word (POS: Noun, Verb, Adverb, Adjective)
	 * Following words of the same POS will be concatenated,
	 * if the new word has a minimum of one synset provided by WordNet
	 * 
	 * @param taggedWords words to tag (filtered by POS)
	 * 
	 * @return Collection of TaggedWords and associated wordsenses(synsets)
	 */
	public static List<MTurkWord> getWordNetInfos(ArrayList<MTurkWord> taggedWords) 
	{
		List<MTurkWord> result = new ArrayList<MTurkWord>();

		MTurkWord lastWord = null;
		
		//Process each word
		for (MTurkWord word : taggedWords) 
		{
			//Convert StanfordTagger POS to WordNet POS
			
			if(lastWord != null)
			{
				//Convert POS
				POS currenttype = WordNet3.convertTagToWordnetPOS(word.getTag());
				POS lasttype = WordNet3.convertTagToWordnetPOS(lastWord.getTag());
				
				//Check if the last and current word have the same POS
				if(lasttype == POS.NOUN || currenttype == POS.NOUN)
				{
					//Concatenate words
					String connectedWord = lastWord.getValue() + " " + word.getValue();
					//Check wordnet
					List<ISynset> wordSynSet = WordNet3.getSynsets(connectedWord,
							POS.NOUN);
					
					//If connectedWord has own synsets at WordNet, use it instead of separate
					if(wordSynSet.size() > 0)
					{
						//Take this one, remove old
						result.remove(lastWord);
			
//						if (printInfos)
//							System.out.println("! Wordnet removed '" + lastWord.getValue() +"' and added '"+connectedWord+"' instead!");
						
						//Update Word
						word = new MTurkWord(connectedWord, WordNet3.PENN_NN,lastWord.getIndex());
					}
				}
			}
			POS type = WordNet3.convertTagToWordnetPOS(word.getTag());
			//Relevant part of speech
			if (type != null) 
			{
				//Collect Synsets
				List<ISynset> wordSynSet = WordNet3.getSynsets(word.getValue(),
						type);

//				if (printInfos)
//					System.out.println("Wordnet resolved '" + word.getValue()
//							+ "' as " + type + ": " + wordSynSet);

				//save synset for target word
				result.add(word);
			}
			
			lastWord = word;
		}

		return result;
	}
}
