package edu.cs.tum.mturk.model;

import java.util.HashMap;

/**
 * This class represents a result of one MTurkQuestion
 * 
 * @author Stephan Epping
 *
 */
public class MTurkQuestionResult {

	private HashMap<String,Integer> answers;
	private String maxAnswer;
	private String questionIdentifier;
	
	/**
	 * Creates a new QuestionResult
	 * 
	 * @param questionId questionIdentifier posted with the question
	 * e.g. JSON string with question information
	 */
	public MTurkQuestionResult(String questionId)
	{
		super();
		this.questionIdentifier = questionId;
		answers = new HashMap<String, Integer>();
		maxAnswer = null;
	}
	
	/**
	 * Counts the answer from a worker
	 * 
	 * @param answerValue a worker selected
	 */
	public void countAnswer(String answerValue) {
		
		if(!answers.containsKey(answerValue))
			answers.put(answerValue, new Integer(0));
		
		//Count answer
		Integer count = answers.get(answerValue);
		count++;
		answers.put(answerValue, count);
		
		//Check if this answer maybe is the new top answer
		if(maxAnswer == null || count > answers.get(maxAnswer))
			maxAnswer = answerValue;
		
	}

	public HashMap<String, Integer> getAnswers() {
		return answers;
	}

	public void setAnswers(HashMap<String, Integer> answers) {
		this.answers = answers;
	}

	public String getMaxAnswer() {
		return maxAnswer;
	}

	public void setMaxAnswer(String maxAnswer) {
		this.maxAnswer = maxAnswer;
	}

	public String getQuestionIdentifier() {
		return questionIdentifier;
	}

	public void setQuestionIdentifier(String questionIdentifier) {
		this.questionIdentifier = questionIdentifier;
	}
}
