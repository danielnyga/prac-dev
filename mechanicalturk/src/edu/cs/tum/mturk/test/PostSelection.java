package edu.cs.tum.mturk.test;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Date;

import com.amazonaws.mturk.addon.HITDataCSVReader;
import com.amazonaws.mturk.addon.HITDataInput;
import com.amazonaws.mturk.addon.HITDataOutput;

import edu.cs.tum.mturk.MTurk;
import edu.cs.tum.mturk.help.MTurkDataOutput;
import edu.cs.tum.mturk.model.MTurkProperties;

/**
 * This class posts all task to MTurk for the sentences provided in the specified file
 * 
 * @author Stephan Epping
 *
 */

public class PostSelection {

	public static final String __PATH = "data/live_2/";
	
	public static final String __VERB = "PLACE";
	/**
	 * @param args
	 */
	public static void main(String[] args) 
	{
		File input = new File(__PATH+"/"+__VERB+"/Sentences.txt");
		
		try 
		{	
			System.setOut(new PrintStream(new FileOutputStream(new File(__PATH+"/"+__VERB+"/"+"post.log"),true),true));
			
			System.out.println(">>> PostSelection "+input+" on "+ new Date());
			MTurk turk = new MTurk();
		
			MTurkProperties properties = new MTurkProperties(__PATH+"HIT.properties");
			HITDataInput secure = new HITDataCSVReader(__PATH+"SECURE.input");
			//Post a new tasks
			
			BufferedReader in = new BufferedReader(new FileReader(input));
			String line = null;
			String header = null;
			HITDataOutput out = null;
			
			int postedHITS = 0;
			
			while((line = in.readLine())!=null)
			{
				ArrayList<String> sentences = new ArrayList<String>();
				
				//Read All sentences
				while(!line.startsWith("__") && line != null)
				{
					sentences.add(line);
					line = in.readLine();
				}
				if(sentences.size() > 0)
				{
					int tasks = turk.createSecureTasks(sentences.toArray(new String[sentences.size()]), header, properties, null, out, 4, secure, 1);
					System.out.println(header+ " posted " + tasks+ " HITs");
					postedHITS += tasks;
					sentences.clear();
					System.out.println("Clear");
				}
					
				if(line.startsWith("__") && !line.equals("__ENDE__"))
				{
					header = line.replaceAll("_", "");
					//header = header.substring(0, header.indexOf(" "));
					
					//create new subdirectory
					String path = __PATH+header+"/";
					File directory = new File(path);
					
					if(!directory.exists())
						directory.mkdir();
					
					//open new outputfile
					out = new MTurkDataOutput(path+"HITS.success");
				}
			}
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println(">>> PostSelection done "+ new Date());
	}

}
