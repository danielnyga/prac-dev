package edu.cs.tum.mturk.test;

import java.io.File;

import com.amazonaws.mturk.addon.HITDataCSVReader;
import com.amazonaws.mturk.addon.HITDataInput;

import edu.cs.tum.mturk.MTurk;

/**
 * This class extends all tasks in the given files
 * 
 * @author Stephan Epping
 *
 */

public class ExtendTasks {

public static final String __PATH = "data/live_2/";
	
	public static final String[] __VERBS = //[{"ADD",
		{"CUT","FILL","FLIP","MIX","PLACE","POUR","PUT","SLICE","STIR"};
	
	public static int [] evaluated = new int[__VERBS.length];
	
	
	public static void main(String[] args) throws Exception
	{
		MTurk turk = new MTurk();
		
		int c = 0;
		for(String verb : __VERBS)
		{
			File in = new File(__PATH+verb+"/HITS.success");
			
			HITDataInput input = new HITDataCSVReader(in.getAbsolutePath());
			
			System.out.println("in: "+in.getCanonicalPath());
			
			turk.extendTasks(input, null, new Long(604800));
			
			c++;
		}
	}
}
