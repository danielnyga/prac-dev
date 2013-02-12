package edu.cs.tum.mturk.test;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * This collects or filters the collected corpus
 * Note: The directory/File structure has to be in a specific pattern
 */

public class CollectAllResults {

	public static final String __PATH = "data/live_2/";
	
	public static final String[] __VERBS = {"ADD","CUT","FILL","FLIP","MIX","PLACE","POUR","PUT","SLICE","STIR"};
	
	public static int [] evaluated = new int[__VERBS.length];
	public static String run = "";
	
	public static void main(String[] args) throws Exception
	{
		
		//collectCorpus();
		
		filterCorpus("let");
		
	}

	/**
	 * This method filters the collected corpus by a single target word
	 * @param searchLemma
	 * @throws FileNotFoundException
	 * @throws ParseException
	 * @throws IOException
	 */
	public static void filterCorpus(String searchLemma)
			throws FileNotFoundException, ParseException, IOException {
		File in = new File(__PATH+ "/corpus.json");
		File out = new File(__PATH+"/corpus_"+searchLemma+".json");
		
		BufferedReader br = new BufferedReader(new FileReader(in));
		JSONArray all = new JSONArray(br.readLine());
		br.close();
		
		JSONArray filtered = new JSONArray();
		
		int count = 0;
		for(int i = 0; i < all.length();i++)
		{
			JSONObject sentence = all.getJSONObject(i);
			
			JSONArray words = sentence.getJSONArray("words");
			
			for(int j = 0; j < words.length(); j++)
			{
				JSONObject word = words.getJSONObject(j);
				
				if(word.getString("lemma").equals(searchLemma))
				{
					filtered.put(sentence);
					System.out.println(sentence.get("context")+" - "+word.getString("sense"));
					count++;
					break;
				}
			}
		}
		
		BufferedWriter bw = new BufferedWriter(new FileWriter(out));
		bw.write(filtered.toString());
		bw.flush();
		bw.close();
		
		System.out.println("Count "+searchLemma+":" + count);
	}

	/**
	 * For all given VERBS this method collects the summarized results
	 * 
	 * @throws FileNotFoundException
	 * @throws ParseException
	 * @throws IOException
	 */
	public static void collectCorpus() throws FileNotFoundException,
			ParseException, IOException {
		File out = new File(__PATH+ "/corpus.json");
		
		JSONArray sentences = new JSONArray();
		
		int count = 0;
		for(String verb : __VERBS)
		{	
			File in = new File(__PATH+verb+"/Results.summarized");
			
			BufferedReader br = new BufferedReader(new FileReader(in));
			JSONArray tmp = new JSONArray(br.readLine());
			br.close();
			
			for(int i = 0; i < tmp.length();i++){
				sentences.put(tmp.get(i));
				count++;
			}
		}
		
		BufferedWriter bw = new BufferedWriter(new FileWriter(out));
		bw.write(sentences.toString());
		bw.flush();
		bw.close();
		
		System.out.println("Count:" + count);
	}
}
