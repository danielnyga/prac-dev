package edu.cs.tum.mturk.help;

import java.text.ParseException;
import java.util.HashMap;
import java.util.List;

import edu.cs.tum.mturk.MTurkWorkerPool;
import edu.cs.tum.mturk.model.MTurkProperties;
import edu.cs.tum.mturk.model.MTurkQuestion;
import edu.cs.tum.mturk.model.MTurkResult;
import edu.cs.tum.mturk.model.MTurkWorker;

public interface IMTurkParser {
	
	//MturkQuestion serialization
	public String serializeQuestion(MTurkQuestion question);
	public MTurkQuestion deserializeQuestion(String s) throws ParseException;
	
	//Used in MTurk
	String parseAnswersKey(List<MTurkQuestion> questions);
	String parseQuestionsForm(List<MTurkQuestion> questions, MTurkProperties properties, IMTurkTagger tagger);

	//Used in MTurkWorkerPool
	HashMap<String,MTurkWorker> parseWorkerPool(String source) throws ParseException;
	String parseWorkerPool(MTurkWorkerPool pool);
	
	//Used in Calling classes
	String parseResults(List<MTurkResult> results) throws ParseException;
}
