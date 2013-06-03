package edu.cs.tum.mturk.test;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.PrintStream;
import java.util.Date;
import java.util.List;

import com.amazonaws.mturk.addon.HITDataCSVReader;
import com.amazonaws.mturk.addon.HITDataInput;
import com.amazonaws.mturk.addon.HITDataOutput;

import edu.cs.tum.mturk.MTurk;
import edu.cs.tum.mturk.MTurkWorkerPool;
import edu.cs.tum.mturk.help.MTurkDataOutput;
import edu.cs.tum.mturk.model.MTurkResult;

/**
 * This class enables the Evaluation of a bunch of posted HITS
 * Note the specific filesystem structure
 * @author Stephan Epping
 *
 */
public class Evaluate {
	
	public static final String __PATH = "data/live_2/";
	
	public static final String[] __VERBS = {"ADD","CUT","FILL","FLIP","MIX","PLACE","POUR","PUT","SLICE","STIR"};
	public static int [] evaluated = new int[__VERBS.length];
	public static String run = "";
	
	public static void main(String[] args) throws Exception
	{
		MTurk turk = new MTurk();
		long time = new Date().getTime();
		System.setOut(new PrintStream(new FileOutputStream(new File(__PATH+"_logs/MTURK_"+time+".log"),true),true));
		
		MTurkWorkerPool pool = new MTurkWorkerPool(__PATH + "WORKER.pool");
		pool.open();
		
		int c = 0;
		for(String verb : __VERBS)
		{
			File directory = new File(__PATH+verb);
			
			//Input HITS
			File in = new File(directory.getAbsolutePath()+"/HITS.success");
			//Output, which HITS could not being processed
			File out = new File(directory.getAbsolutePath()+ "/HITS.notprocessed");
			
			HITDataInput input = new HITDataCSVReader(in.getAbsolutePath());
			HITDataOutput output = new MTurkDataOutput(out.getAbsolutePath());
			
			System.out.println("in: "+in.getCanonicalPath());
			System.out.println("out: "+out.getCanonicalPath());
			
			//Run Evaluation
			List<MTurkResult> results = turk.evaluateTasks(input, output, pool);
			evaluated[c]= results.size();
			
			//Parse Results to JSON
			String json = turk.getTurkParser().parseResults(results);
			
			//Save on disk
			FileWriter f = new FileWriter(directory + "/HITS.results");
			f.write(json);
			f.flush();
			f.close();
			
			output.close();
			c++;
		}
		
		pool.close();
		
		System.out.println("---- SUMMARY ----");
		for(int i = 0; i < evaluated.length; i++)
			System.out.println(__VERBS[i]+" evaluated "+evaluated[i]);
	}
}
