package edu.cs.tum.mturk.help;

import java.io.IOException;
import java.util.HashMap;

import com.amazonaws.mturk.addon.HITDataInput;
import com.amazonaws.mturk.addon.HITDataOutput;
import com.amazonaws.mturk.addon.HITProperties;
import com.amazonaws.mturk.requester.HIT;
import com.amazonaws.mturk.requester.QualificationType;

/**
 * This class provides a few helper methods
 * 
 * @author Stephan Epping
 *
 */

public final class MTurkHelper {

	/**
	 * Writes HIT infos into a file
	 * 
	 * @param output output file
	 * @param hit specific HIT
	 * @param comment to append to HIT infos
	 * 
	 * @throws IOException
	 */
	public static void writeInfos(HITDataOutput output,HIT hit,String comment) throws IOException
	{
		if(output == null){
			printHIT(hit);
		}else{
			HashMap<String, String> infos = new HashMap<String, String>();
			infos.put(HITProperties.HITField.HitId.getFieldName(), hit
					.getHITId());
			infos.put(HITProperties.HITField.HitTypeId.getFieldName(), hit
					.getHITTypeId());
			infos.put("Comment", comment);
			
			output.writeValues(infos);
		}
	}
	
	/**
	 * Validates the input for task processing
	 * 
	 * @param input file with HITs
	 * 
	 * @return true, if the input file has a valid format
	 */
	public static boolean validateInputForTasks(HITDataInput input){
		
		if(input.getFieldNames().length < 3)
			return false;
		
		for(int i = 0; i < 3; i++)
		{
			switch (i) {
			case 0:
				if(!input.getFieldNames()[0].equals(HITProperties.HITField.HitId.getFieldName()))
					return false;
				break;
			case 1:
				if(!input.getFieldNames()[1].equals(HITProperties.HITField.HitTypeId.getFieldName()))
					return false;
				break;
			case 2:
				if(!input.getFieldNames()[2].equals("Comment"))
					return false;
				break;
			default:
				break;
			}
		}
		return true;
	}
	
	/**
	 * Validates the input for qualification tests
	 * 
	 * @param input file with Test data
	 * 
	 * @return true, if the input file has a valid format
	 */
	public static boolean validateInputForQualification(HITDataInput input){
		
		if(input.getFieldNames().length < 3)
			return false;
		
		for(int i = 0; i < 3; i++)
		{
			switch (i) {
			case 0:
				if(!input.getFieldNames()[0].equals("sentence"))
					return false;
				break;
			case 1:
				if(!input.getFieldNames()[1].equals("word"))
					return false;
				break;
			case 2:
				if(!input.getFieldNames()[2].equals("answer"))
					return false;
				break;
			default:
				break;
			}
		}
		return true;
	}
	
	/**
	 * Prints infos of a qualification test on console
	 * 
	 * @param t qualification test
	 */
	public static void printQualification(QualificationType t) {
		System.out.println(""+t.getName()+" "+t.getDescription()+" "+t.getQualificationTypeId());	
	}
	
	/**
	 * Prints information of a HIT on console
	 * 
	 * @param hit task
	 */
	public static void printHIT(HIT hit) {
		
		String output = hit.getHITId() ;
		
		if(hit.getTitle() != null)
			output += " '"+ hit.getTitle() + "'";
		
		if(hit.getRequesterAnnotation() != null)
				output += " [ANNOTATION "+ hit.getRequesterAnnotation()+"]";
		
		if(hit.getHITTypeId() != null)
				output += " [TYPE " + hit.getHITTypeId() + "]";
		
		if(hit.getHITStatus() != null)
			output += " [STATE " + hit.getHITStatus() + "/" + hit.getHITReviewStatus() + "]";
			
		if(hit.getMaxAssignments() != null)
			output += " [M:" + hit.getMaxAssignments() + "/A:" + hit.getNumberOfAssignmentsAvailable() + "/P:" + hit.getNumberOfAssignmentsPending() + "/C:" + hit.getNumberOfAssignmentsCompleted() + "]";
		
		if(hit.getExpiration() != null)
			output += " [EXPIRATION " + hit.getExpiration().getTime() + "]";
		
	
		System.out.println("HIT "+output);
	}
	
}
