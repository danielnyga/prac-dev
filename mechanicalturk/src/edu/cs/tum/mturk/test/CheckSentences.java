package edu.cs.tum.mturk.test;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.ArrayList;

import edu.cs.tum.mturk.MTurkExtended;

/**
 * This class check all lines in a Document for duplicates, where "__" denotes a new section
 * and some statistics are printed
 */

public class CheckSentences {

	public static void main(String[] args) 
	{
		String TARGET = "data/WORDS/";
		
		try 
		{
			//Logfile
			System.setOut(new PrintStream(TARGET+"1__log.txt"));
			
			//Input file
			BufferedReader in = new BufferedReader(new FileReader(TARGET+"1__SELECTED__.txt"));
			
			String line = null;
			
			MTurkExtended mturk = new MTurkExtended();
			
			ArrayList<String> list = new ArrayList<String>();
			
			ArrayList<String> lines = new ArrayList<String>();
			
			int count = 0;
			String header = null;
			while((line = in.readLine()) != null)
			{
				if(line.startsWith("__"))
				{
					//skip its a new word
					if(count!=0)
					{
						System.out.println("Containing Lines: "+count);
						System.out.println("Number of HITs["+header+"]: "+mturk.checkNumberOfHITs(lines.toArray(new String[lines.size()]), 4));
					}
					
					header = line;
					
					System.out.println("Check "+line);
					count = 0;
					lines.clear();
					
				}else{
					if(list.contains(line))
						System.out.println("Duplicate: "+line);
					else
					{
						count++;
						list.add(line);
						lines.add(line);
					}
				}
			}
			
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

}
