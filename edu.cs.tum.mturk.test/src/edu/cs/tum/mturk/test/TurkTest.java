package edu.cs.tum.mturk.test;

import java.util.Date;

import com.amazonaws.mturk.addon.HITDataCSVReader;
import com.amazonaws.mturk.addon.HITDataInput;
import com.amazonaws.mturk.addon.HITDataOutput;

import edu.cs.tum.mturk.MTurk;
import edu.cs.tum.mturk.MTurkExtended;

import edu.cs.tum.mturk.model.MTurkProperties;

/**
 * This class demonstrates the general usage of MTurk
 * 
 * @author Stephan Epping
 *
 */
public class TurkTest {

	public static final String __PATH = "DATA/live_1/";
	
	public static void main(String[] args) {
		
		try 
		{
			//System.setOut(new PrintStream(new FileOutputStream(new File(__PATH + "MTURK.log"),true),true));
			System.out.println(">>> New Session "+ new Date());
			
			//MTurk t = new MTurk();
			
			//t.sendNotification("A2EPJ9V5DYMHR2", "In the sentence 'Pour batter into greased and floured round cake pans, 1 color per pan.' the word round is NOT a Noun therefore you should answer 'Other part of speech'", "Tip");
			/*
			IMTurkTagger pos = new StanfordPOSParser();
			MTurkParser p = new MTurkParser();
			//String sentence = "Pour blueberry mixture into prepared pie crust.";
			String sentence = "Pour approximately 2 tablespoons of extra virgin olive oil into the bottom of the baking dish or roasting pan.";
			for(MTurkWord w : pos.tag(sentence))
				System.out.println(w.getValue()+" "+w.getIndex()+" "+w.getTag());
			*/
			
			//MTurkWord word = pos.tag(sentence).get(9);
			//System.out.println(""+p.highlightWord(word, sentence, pos));
			
			
			MTurkExtended ex = new MTurkExtended();
			//ex.printAllTasks();
			//ex.printScores("24XEGDHDM25LXSCTSTXHHZ6OE5YDNV");
			ex.printScores("29IPSI2KYEPLHZABDID61J3OAIBSMJ");
			//LIVE 29IPSI2KYEPLHZABDID61J3OAIBSMJ
			//ex.printAllTasks();
			
			//TurkTest.cleanAll();
			//TurkTest.postQualification();
			//TurkTest.postTasks();
			//TurkTest.postSecureTasks();
			//TurkTest.evaluate(false);
			//TurkTest.expire();
			//TurkTest.extend();
		} 
		catch (Exception e) 
		{
			e.printStackTrace();
		}
		System.out.println(">>> Session done "+ new Date());
	}
	
	public static void extend() throws Exception
	{
		MTurk turk = new MTurk();
		
		HITDataInput input = new HITDataCSVReader("test/task_run1.notprocessed");
		
		turk.extendTasks(input,null,new Long(60*10));//Extends Task +1 Assignments + 60 seconds time
	}
	
	public static void expire() throws Exception
	{
		MTurk turk = new MTurk();
		
		HITDataInput input = new HITDataCSVReader("test/pour_live_1.success");
		
		turk.expireTasks(input);
	}

	public static void postTasks() throws Exception
	{	
		MTurk turk = new MTurk();
		
		MTurkProperties properties = MTurkProperties.getDefaults();
		properties = new MTurkProperties("task_std1.properties");
		
		//Post a new tasks
		HITDataOutput logfile = null;//new MTurkDataOutput("test/task_run1.success");
		String hittypeid = null;//use automatic taskgroup
		
		turk.createTasks(new String[]{"Place the cup under the coffee maker"}, "test", properties, hittypeid, logfile, 3);
		//turk.createTasks(new String[]{"Push the spatula under the pancake"}, "test",properties, hittypeid, logfile, 3);
	}
	
	public static void postSecureTasks() throws Exception
	{	
		MTurk turk = new MTurk();
		
		MTurkProperties properties = MTurkProperties.getDefaults();
		properties = new MTurkProperties("task_std1.properties");
		
		//Post a new tasks
		HITDataOutput logfile = null;//new MTurkDataOutput("test/task_run1.success");
		String hittypeid = null;//use automatic taskgroup
		
		HITDataInput secureInput = new HITDataCSVReader("qualification1.input");
		
		turk.createSecureTasks(new String[]{"Place the cup under the coffee maker"}, "test", properties, hittypeid, logfile, 3,secureInput,2);
		//turk.createTasks(new String[]{"Push the spatula under the pancake"}, properties, hittypeid, logfile, 3);
	}
	
	public static void postQualification() throws Exception
	{	
		MTurk turk = new MTurk();
		
		MTurkProperties properties = new MTurkProperties("qualification_pour.properties");
		HITDataInput input = new HITDataCSVReader("qualification_pour.input");
		
		turk.createQualification(input, properties);
	}
	
	public static void cleanAll() throws Exception
	{
		MTurkExtended turk = new MTurkExtended();
		
		HITDataInput input = new HITDataCSVReader(__PATH + "HITS.success");
		turk.deleteTasks(input);
		//turk.deleteAllQualifications();
		//turk.deleteAllTasks();
	}
}
