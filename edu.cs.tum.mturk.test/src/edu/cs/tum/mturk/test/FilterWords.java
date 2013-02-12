package edu.cs.tum.mturk.test;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * This class searches the wikihow files for specific words
 * and save all sentences in a txt file
 * 
 * @author Stephan Epping
 *
 */

public class FilterWords {

	public static void main(String[] args) 
	{	
		String ROOT = "../../Wikihow";
		String TARGET = "data/WORDS/";
		
		File relevant = new File(ROOT + "/selected.txt");

		//fill,pour,add,cut,slice,flip,stir,mix,make,cook
		String[] relevantWords = new String[]{
				"fill","filling","filled","fills",
				"pour","pouring","poured","pours",
				"add", "adding","added","adds",
				"cut", "cutting","cuts",
				"slice","slicing","sliced","slices",
				"flip","flipping","flipped","flips",
				"stir","stirring","stirred","stirs",
				"mix","mixing","mixed","mixt","mixes",
				"make","making","made","makes",
				"cook","cooking","cooked","cooks",
				"place","put"
				};
		try 
		{
			BufferedReader relevantFiles = new BufferedReader(new FileReader(relevant));
			HashMap<String, BufferedWriter> writer = new HashMap<String, BufferedWriter>();
			
			int files = 0;
			String singleFile = null;
			while((singleFile=relevantFiles.readLine())!=null)
			{
				files++;
				File current = new File(ROOT +"/"+ singleFile);
				System.out.println("Next File "+ singleFile);
				
				String[] fileContent = readFile(current);
				
				for(String sentence : fileContent)
				{
					for(String word : relevantWords)
					{
						if(sentence.toLowerCase().contains(word.toLowerCase()+" "))
						{
							if(!writer.containsKey(word))
								writer.put(word, new BufferedWriter(new FileWriter(TARGET+""+word.trim()+".txt")));
							
							writer.get(word).write(sentence+"\n");
							System.out.println(" "+word+" : "+sentence);
						}
						
					}
					System.out.println(" > done : "+sentence);
				}
					
			}
			
			System.out.println("Files done: "+files);
			
			for(BufferedWriter w :writer.values()){
				w.flush();
				w.close();
			}
			
			relevantFiles.close();
			
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private static String[] readFile(File f) throws IOException
	{
		BufferedReader fileContent = new BufferedReader(new FileReader(f));
		
		List<String> sentences = new ArrayList<String>();
		String sentence = null;
		
		while((sentence=fileContent.readLine())!=null)
			sentences.add(sentence);
		
		fileContent.close();
		
		return (String[])sentences.toArray(new String[sentences.size()]);
	}
}
