package edu.cs.tum.mturk;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.amazonaws.mturk.addon.HITDataInput;
import com.amazonaws.mturk.addon.HITProperties;
import com.amazonaws.mturk.requester.HIT;
import com.amazonaws.mturk.requester.HITStatus;
import com.amazonaws.mturk.requester.Qualification;
import com.amazonaws.mturk.requester.QualificationType;

import edu.cs.tum.mturk.help.MTurkHelper;
import edu.cs.tum.mturk.model.MTurkQuestion;
import edu.cs.tum.mturk.model.MTurkWord;
import edu.mit.jwi.item.ISynset;

/**
 * This class extends the standard MTurk by adding a few more convenient methods
 * like printing general information or cleaning up (Sandbox!)
 * 
 * @author Stephan Epping
 *
 */
public class MTurkExtended extends MTurk {

	public MTurkExtended() throws IOException {
		super();
	}
	
	/**
	 * Prints information about all posted qualification tests
	 */
	public void printAllQualifications()
	{	
		for(QualificationType qual : service.getAllQualificationTypes())
			MTurkHelper.printQualification(qual);
	}
	
	/**
	 * Prints all information about a single task
	 * 
	 * @param hitId of the specific task
	 */
	public void printTask(String hitId)
	{
		HIT hit = service.getHIT(hitId);
		MTurkHelper.printHIT(hit);
	}
	
	/**
	 * Prints the scores of a qualification test
	 * 
	 * @param qualificationTypeId
	 * @throws Exception
	 */
	public void printScores(String qualificationTypeId) throws Exception
	{
		
		int passed = 0;
		int all = 0;
		
		//TODO while(quals!=null)...
		for(int i = 1;i<100;i++)
		{	
			Qualification[] quals = service.getQualificationsForQualificationType(qualificationTypeId,i);
			
			if(quals == null)
				break;
			
			for(Qualification q : quals)
			{
				all++;
				System.out.println(q.getSubjectId()+" "+q.getIntegerValue()+" "+q.getGrantTime().getTimeInMillis());
				
				if(q.getIntegerValue() > 6)
					passed++;
			}
		}
		
		System.out.println(">> Summary: ("+passed+"/"+all+")");
	}

	/**
	 * Prints a list of reviewable Tasks (all assignments done or time expired)
	 * 
	 * @param hitTypeId only possible with a specific hitgroup
	 */
	public void printReviewableTasks(String hitTypeId) 
	{
		HIT[] revHits = service.getAllReviewableHITs(hitTypeId);
		
		System.out.println("Reviewable HITS: ");
		for(HIT hit : revHits)
			MTurkHelper.printHIT(service.getHIT(hit.getHITId()));
	}

	/**
	 * Prints a list of all tasks
	 * 
	 * TODO perhaps use specific State!
	 */
	public void printAllTasks() 
	{
		HIT[] allHits = service.searchAllHITs();//Returns ALL HITs
		
		System.out.println("ALL HITS: ");
		for(HIT hit : allHits)
			MTurkHelper.printHIT(hit);
	}
	
	/**
	 * Deletes all qualifications
	 * (the name of a deleted qualification is blocked forever)
	 */
	public void deleteAllQualifications()
	{	
		for(QualificationType qual : service.getAllQualificationTypes())
			this.deleteQualification(qual.getQualificationTypeId());
	}
	
	/**
	 * Deletes a specified qualification test
	 * (the name of a deleted qualification is blocked forever)
	 * 
	 * @param qualificationTypeId to identify test
	 */
	public void deleteQualification(String qualificationTypeId)
	{
		service.disposeQualificationType(qualificationTypeId);
		System.out.println("Cleaned qualification "+qualificationTypeId);
	}
	
	/**
	 * Deletes all TASKS (SANDBOX!)
	 * (Not Reviewed assignments WILL BE APPROVED AND WORKER GETS PAID)
	 */
	public void deleteAllTasks()
	{
		HIT[] allHits = service.searchAllHITs();
		
		for(HIT hit : allHits)
			deleteTask(hit.getHITId());
	}
	
	/**
	 * Deletes all specified Tasks
	 * (Not Reviewed assignments WILL BE APPROVED AND WORKER GETS PAID)
	 * 
	 * @param input list of tasks
	 */
	public void deleteTasks(HITDataInput input)
	{
		int size = input.getNumRows();
		
		//Skip header
		for(int i = 1;i < size; i++)
			deleteTask(input.getRowAsMap(i).get(HITProperties.HITField.HitId.getFieldName()));
	}

	/**
	 * Deletes specified task
	 * (Not Reviewed assignments WILL BE APPROVED AND WORKER GETS PAID)
	 * 
	 * @param hitId specifies task
	 */
	public void deleteTask(String hitId) 
	{
		HIT hit = service.getHIT(hitId);
		
		System.out.println("Cleaned "+hit.getHITId() +" Details: "+ hit.getRequesterAnnotation());
		
		if(hit.getHITStatus() == HITStatus.Assignable)
			service.forceExpireHIT(hit.getHITId());
		
		service.disableHIT(hit.getHITId());
	}
	
	/**
	 * Calculates the number of tasks that would be added to MTurk
	 * 
	 * @param sentences input sentences
	 * @param questionInTask number of questions in tasks
	 * @return number of tasks
	 * @throws Exception
	 */
	public int checkNumberOfHITs(String[] sentences, int questionInTask) throws Exception 
	{
		List<MTurkQuestion> questions = new ArrayList<MTurkQuestion>();
		
		int qCounter = 0;
		int numberOfTasks = 0;
		
		int sentenceIndex = 0;
		for (String sentence : sentences) 
		{
			// Tag words with stanford POS Tagger
			ArrayList<MTurkWord> taggedWords = tagger.tag(sentence);
			
			// Resolve words with wordnet
			HashMap<MTurkWord, List<ISynset>> resolvedWords = getWordNetInfos(taggedWords);

			
			for (MTurkWord word : resolvedWords.keySet()) 
			{
				qCounter++;
				
				//Generate MTurkQuestion
				questions.add(generateQuestion(word, sentence, resolvedWords.get(word), sentenceIndex, "DUMMY"));
				
				if(qCounter == questionInTask)
				{	
					//System.out.println("Post new HIT "+ numberOfTasks+" (Questions: "+questions.size()+" // Current Sentence: "+sentenceIndex+")");
					numberOfTasks++;
					
					//Reset
					qCounter = 0;
					questions.clear();
				}
			}
			
			sentenceIndex++;
		}
		
		//Do remaining questions
		if(questions.size() > 0)
		{
			//System.out.println("Post new HIT "+ numberOfTasks+" (Questions: "+questions.size()+" // Current Sentence: "+sentenceIndex+")");
			numberOfTasks++;
			
			//Reset
			qCounter = 0;
			questions.clear();
		}
		
		return numberOfTasks;
	}
}
