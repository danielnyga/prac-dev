package edu.cs.tum.mturk.help;

import java.io.File;
import java.util.Arrays;

import com.amazonaws.mturk.addon.HITDataCSVWriter;
import com.amazonaws.mturk.addon.HITProperties;

/**
 * This class extends the standard HITDataCSVWriter
 * it appends only data to an existing file, instead of adding the headline again
 * 
 * @author Stephan Epping
 *
 */
public class MTurkDataOutput extends HITDataCSVWriter {

	private static final String[] FIELDS = new String[]{HITProperties.HITField.HitId.getFieldName(),HITProperties.HITField.HitTypeId.getFieldName(),"Comment"};
	
	public MTurkDataOutput(String fileName) 
	{
		super(fileName);
        
		//Check if it is an existing file
		if(!(new File(fileName)).exists())
			//Create headerline
			this.setFieldNames(FIELDS);
		else {
			//just append data
			this.setFieldNames(Arrays.asList(FIELDS));
		}
	}

}
