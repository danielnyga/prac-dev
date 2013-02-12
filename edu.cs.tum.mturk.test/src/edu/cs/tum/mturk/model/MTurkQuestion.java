package edu.cs.tum.mturk.model;

import java.util.List;

/**
 * This class represents one WSD Mturk Question
 * 
 * @author Stephan Epping
 *
 */
public class MTurkQuestion {
	
	private MTurkWord word;
	
	private String sentence;
	private String answer;
	
	private String context;
	
	private List<String> valueTexts;
	private List<String> displayTexts;
	
	/**
	 * Create a new WSD question
	 * 
	 * @param word target word
	 * @param sentence context
	 * @param context 
	 */
	public MTurkQuestion(MTurkWord word, String sentence, String context) {
		super();
		
		this.word = word;
		this.sentence = sentence;
		this.context = context;
	}

	public MTurkWord getWord() {
		return word;
	}

	public void setWord(MTurkWord word) {
		this.word = word;
	}

	public String getSentence() {
		return sentence;
	}

	public void setSentence(String sentence) {
		this.sentence = sentence;
	}

	public List<String> getValueTexts() {
		return valueTexts;
	}

	public void setValueTexts(List<String> valueTexts) {
		this.valueTexts = valueTexts;
	}

	public List<String> getDisplayTexts() {
		return displayTexts;
	}

	public void setDisplayTexts(List<String> displayTexts) {
		this.displayTexts = displayTexts;
	}

	public String getAnswer() {
		return answer;
	}

	public void setAnswer(String answer) {
		this.answer = answer;
	}

	public String getContext() {
		return context;
	}

	public void setContext(String context) {
		this.context = context;
	}
}
