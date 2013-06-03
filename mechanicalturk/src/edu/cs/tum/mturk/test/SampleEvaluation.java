package edu.cs.tum.mturk.test;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.InputStreamReader;
import java.util.List;

import edu.mit.jwi.item.ISynset;
import edu.mit.jwi.item.POS;
import edu.tum.cs.ias.wordnet.WordNet3;

/**
 * Tool to evaluate the results of MTurk easily
 * First the results have to be sampled (SampleResults)
 * 
 * @author Stephan Epping
 *
 */

public class SampleEvaluation {

	/**
	 * @param args
	 */
	public static void main(String[] args) throws Exception
	{
		
		File input = new File("data/live_2/samples.csv");
		File output = new File("data/live_2/samples_out3.csv");

		BufferedReader in = new BufferedReader(new FileReader(input));
		BufferedWriter out = new BufferedWriter(new FileWriter(output));
		
		BufferedReader console = new BufferedReader(new InputStreamReader(System.in));
		int columnSize = 7;
		String line = "";
		while((line = in.readLine())!=null)
		{
			String[] columns = line.split(";");
			
			while(columns.length > columnSize)
				System.out.println("Skip");
			
			String sentence = columns[0];
			String word = columns[1];
			String index = columns[3];
			String tag = columns[2];
			String all = columns[4];
			String nAll = columns[5];
			String rand = columns[6];
			
			POS pos = WordNet3.convertTagToWordnetPOS(tag);
			
			List<ISynset> syns = WordNet3.getSynsets(word, pos);
			
			System.out.println("---------------------------------");
			System.out.println(sentence);
			System.out.println(word+"("+index+"/"+tag+")");
			System.out.println("All["+nAll+"]: "+all+" Random: "+rand);
			
			int i = 0;
			for(ISynset s : syns)
			{
				System.out.println("  "+i+":"+s.getID()+" "+s.getGloss());
				i++;
			}
			
			String cmd = console.readLine();
			
			if(cmd.equals("ende"))
				break;
			
			if(cmd.equals("o"))
				out.write(sentence+";"+word+";"+tag+";"+index+";"+all+";"+rand+";OTHERTYPE\n");
			else{
				if(cmd.equals("n"))
					out.write(sentence+";"+word+";"+tag+";"+index+";"+all+";"+rand+";NOMATCH\n");
				else{
					int right = Integer.parseInt(cmd);
					
					String id = syns.get(right).getID().toString();
					
					out.write(sentence+";"+word+";"+tag+";"+index+";"+all+";"+rand+";"+id+"\n");
				}
			}
		}
		
		in.close();
		console.close();
		
		out.flush();
		out.close();
	}

}
