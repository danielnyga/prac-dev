package edu.cs.tum.mturk.model;

import java.text.ParseException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import com.amazonaws.mturk.dataschema.QuestionFormAnswers;
import com.amazonaws.mturk.dataschema.QuestionFormAnswersType;
import com.amazonaws.mturk.requester.Assignment;
import com.amazonaws.mturk.requester.HIT;
import com.amazonaws.mturk.service.axis.RequesterService;

import edu.cs.tum.mturk.MTurkWorkerPool;
import edu.cs.tum.mturk.help.IMTurkParser;

/**
 * This class represents the result of a whole HIT
 * it evaluates the result by analyzing the assignments
 * it can contain multiple MTurkQuestionResults 
 * 
 * @author Stephan Epping
 *
 */
public class MTurkResult 
{
	private HIT hit;
	private Assignment[] assignments;
	
	private HashMap<String, MTurkQuestionResult> results;

	IMTurkParser parser;
	
	/**
	 * Evaluates the assignments of a specific task
	 * 
	 * @param hit
	 * @param assignments
	 * @throws Exception 
	 */
	public MTurkResult(HIT hit, Assignment[] assignments, IMTurkParser parser, MTurkWorkerPool pool) throws Exception 
	{
		super();
		this.hit = hit;
		this.assignments = assignments;
		this.parser = parser;
		
		analyze(assignments);
	}
	
	public Collection<MTurkQuestionResult> getResults()
	{
		return results.values();
	}
	
	public MTurkQuestionResult getResult(String questionID)
	{
		return results.get(questionID);
	}
	
	@SuppressWarnings("unchecked")
	public static List<QuestionFormAnswersType.AnswerType> getAnswers(Assignment assignment){
		
		QuestionFormAnswers qfa = RequesterService.parseAnswers(assignment.getAnswer());
		
		return (List<QuestionFormAnswersType.AnswerType>) qfa.getAnswer();
	}
	
	private void analyze(Assignment[] assignments) throws ParseException 
	{
		results = new HashMap<String, MTurkQuestionResult>();
		
		//Process assignments
		for(Assignment assignment : assignments)
		{	
			String assignmentId = assignment.getAssignmentId();
			
			//Process each question
			for (QuestionFormAnswersType.AnswerType answer : getAnswers(assignment)) 
			{
				String questionID = answer.getQuestionIdentifier();
				MTurkQuestion question = parser.deserializeQuestion(answer.getQuestionIdentifier());
					
				if(question.getAnswer()==null)
				{
					//Get current answer
			    	String answerValue = RequesterService.getAnswerValue(assignmentId, answer);
			        
			    	//Check if this answer was given by another assignment
			    	if(!results.containsKey(questionID))
			    		results.put(questionID, new MTurkQuestionResult(questionID));
			    	
			    	MTurkQuestionResult questionResult = results.get(questionID);
			    	
			    	//Count answer
			    	questionResult.countAnswer(answerValue);
				}
			}
		}
	}

	public HIT getHit() {
		return hit;
	}

	public Assignment[] getAssignments() {
		return assignments;
	}
}
