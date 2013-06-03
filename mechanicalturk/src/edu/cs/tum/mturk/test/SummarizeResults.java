package edu.cs.tum.mturk.test;

import java.io.*;
import java.util.*;

import org.json.*;

import edu.cs.tum.mturk.pos.parser.Parser;
import edu.mit.jwi.item.POS;
import edu.stanford.nlp.trees.TypedDependency;
import edu.tum.cs.ias.wordnet.WordNet3;

/**
 * This class groups the single HIT results by sentence and enriches them with some extra features
 * 
 * @author Stephan Epping
 *
 */

public class SummarizeResults {

	public static String __results = "data/live_2/STIR/";
	
	public static void main(String[] args) 
	{	
		try 
		{
			//Enrich Features with dependencies
			Parser parser = new Parser("lib/englishPCFG.ser.gz");
			
			BufferedReader input = new BufferedReader(new FileReader(__results+"HITS.results"));
			String jsonstring = input.readLine();
			input.close();
			
			JSONArray results = new JSONArray(jsonstring);
			
			HashMap<String,List<JSONObject>> summary = new HashMap<String,List<JSONObject>>();
			
			//Collect Results by Sentence
			for(int i = 0; i < results.length();i++)
			{
				JSONObject annotation = results.getJSONObject(i);
				
				String sentence = annotation.getString("sentence");
				
				if(!summary.containsKey(sentence))
					summary.put(sentence,new ArrayList<JSONObject>());
				
				summary.get(sentence).add(annotation);
			}
			
			//Summarize Sentence
			JSONArray summarized = new JSONArray();
			JSONArray failed = new JSONArray();
			
			for(String sentence : summary.keySet())
			{
				List<JSONObject> words = summary.get(sentence);
				
				//Gather dependencies
				ArrayList<TypedDependency> dependencies = getRelevantDependencies(
						parser, sentence);
								
				System.out.println(dependencies);
				
				JSONObject jSentence = new JSONObject();
				jSentence.put("context", sentence);
				
				JSONArray jWords = new JSONArray();
				jSentence.put("words", jWords);
				
				for(JSONObject word : words)
				{
					String sense = getResult(word);
					
					if(sense != null)
					{
						JSONObject jWord = new JSONObject();
						jWords.put(jWord);
						
						jWord.put("word", word.get("word"));
						jWord.put("index", word.get("index"));
						jWord.put("tag", word.get("tag"));
						jWord.put("sense", sense);
						enrichFeature(jWord,dependencies);
						
					}else{
						failed.put(word);
					}
				}
				
				if(jWords.length() > 0)
					summarized.put(jSentence);
			}
			
			BufferedWriter output = new BufferedWriter(new FileWriter(__results+"Results.summarized"));
			output.write(summarized.toString());
			output.flush();
			output.close();
			
			BufferedWriter output2 = new BufferedWriter(new FileWriter(__results+"Results.failed"));
			output2.write(failed.toString());
			output2.flush();
			output2.close();
			
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	private static ArrayList<TypedDependency> getRelevantDependencies(
			Parser parser, String sentence) {
		ArrayList<TypedDependency> dependencies = new ArrayList<TypedDependency>();
		
		Collection<TypedDependency> cdependencies = parser.getCollapsedDependencies(sentence);
		Iterator<TypedDependency> it = cdependencies.iterator();
		while(it.hasNext())
		{
			TypedDependency d = it.next();
			
			if(d.reln().getShortName().equals("prep"))
				dependencies.add(d);
			
			if(d.reln().getShortName().equals("amod"))
				dependencies.add(d);
		}
		return dependencies;
	}
	
	private static String getResult(JSONObject annotation)
	{
		JSONArray results = annotation.getJSONArray("results");
		
		int maxValue = -1;
		String maxSense = null;
		int maxSenses = 0;
		
		for(int i = 0; i < results.length();i++)
		{
			JSONObject result = results.getJSONObject(i);
			
			String sense = result.getString("sense");
			int votes = result.getInt("votes");
			
				if(votes > maxValue){
					maxValue = votes;
					maxSense = sense;
					maxSenses = 1;
				}else{
					if(votes == maxValue)
						maxSenses++;
				}
		}
		
		//sense it not useable, no appropriate sense tagged
		if(maxSense.equals("NOMATCH") || maxSense.equals("OTHERTYPE"))
			return null;
		
		//no unique sense found
		if(maxSenses > 1)
			return null;
		else
			return maxSense;
	}
	
	private static void enrichFeature(JSONObject jWord, List<TypedDependency> dependencies)
	{
		POS pos = WordNet3.convertTagToWordnetPOS(jWord.getString("tag"));
		String lemma = WordNet3.getWordStem(jWord.getString("word"), pos);
		int index = jWord.getInt("index");
		//Enrich
		jWord.put("pos", Character.toString(pos.getTag()));
		jWord.put("lemma", lemma);
		
		//Compound word
		if(jWord.getString("word").contains(" "))
			jWord.put("cindex", index+1);
			
		
		JSONArray jDeps = new JSONArray();
		for(TypedDependency dep : dependencies)
		{
			if(dep.gov().index()-1==index || dep.dep().index()-1==index)
			{
				JSONObject jDep = new JSONObject();
				jDeps.put(jDep);
				jDep.put("relation",dep.reln().getShortName());
				
				if(dep.reln().getSpecific()!=null)
					jDep.put("specific", dep.reln().getSpecific());
			}
		}
		
		if(jDeps.length() > 0)
			jWord.put("dependecies", jDeps);
	}

}
