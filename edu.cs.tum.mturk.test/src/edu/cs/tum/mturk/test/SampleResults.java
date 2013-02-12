package edu.cs.tum.mturk.test;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * This class randomly selects a number of results
 * 
 * @author Stephan Epping
 *
 */

public class SampleResults {

	public static final String[] __VERBS = {"ADD","CUT","FILL","FLIP","MIX","PLACE","POUR","PUT","SLICE","STIR"};
	public static final String __PATH = "data/live_2/";
	
	public static int samplesize = 400;
	
	public static void main(String[] args) throws Exception 
	{
		
		List<JSONObject> annotations = new ArrayList<JSONObject>();
		
		BufferedWriter bw = new BufferedWriter(new FileWriter(new File(__PATH+"samples.csv")));
		
		for(String verb : __VERBS)
		{
			File directory = new File(__PATH+verb);
			
			File result = new File(directory.getAbsolutePath()+"/HITS.results");

			BufferedReader br = new BufferedReader(new FileReader(result));
			
			JSONArray results = new JSONArray(br.readLine());
			
			for(int i = 0;i < results.length();i++){
				annotations.add(results.getJSONObject(i));
			}
			
			System.out.println("Read "+verb+":"+results.length());
			
			br.close();
		}
		System.out.println("Annotations:"+annotations.size());
		
		Random rand = new Random();
		
		int sampled = 0;
		while(sampled < samplesize)
		{
			int an = rand.nextInt(annotations.size());
			System.out.println("Sampled: "+an);
			JSONObject annotation = annotations.get(an);
			
			String sentence = annotation.getString("sentence");
			String word = annotation.getString("word");
			String tag = annotation.getString("tag");
			int index = annotation.getInt("index");
			
			JSONArray all = annotation.getJSONArray("results");
			
			String allSaid = getMaxAnswer(all);
			
			int votes = 0;
			for(int i = 0; i < all.length();i++)
				votes += all.getJSONObject(i).getInt("votes");
			
			String randomSaid = all.getJSONObject(rand.nextInt(votes)%all.length()).getString("sense");
		
			bw.write(sentence+";"+word+";"+tag+";"+index+";"+allSaid+";"+randomSaid+";\n");
			sampled++;
		}
		
		bw.flush();
		bw.close();
		
	}
	
	public static String getMaxAnswer(JSONArray r)
	{
		String maxAnswer = null;
		int maxAnswers = -1;
		
		for(int i = 0; i < r.length();i++)
		{
			int v = r.getJSONObject(i).getInt("votes");
			
			if(v > maxAnswers)
			{
				maxAnswer = r.getJSONObject(i).getString("sense");
				maxAnswers = v;
			}
		}
		
		return maxAnswer+";"+maxAnswers;
	}

}
