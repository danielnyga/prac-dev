package edu.cs.tum.mturk.help;

import java.text.ParseException;
import java.util.HashMap;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.cs.tum.mturk.MTurkWorkerPool;
import edu.cs.tum.mturk.model.MTurkProperties;
import edu.cs.tum.mturk.model.MTurkQuestion;
import edu.cs.tum.mturk.model.MTurkQuestionResult;
import edu.cs.tum.mturk.model.MTurkResult;
import edu.cs.tum.mturk.model.MTurkWord;
import edu.cs.tum.mturk.model.MTurkWorker;
import edu.mit.jwi.item.POS;

import edu.tum.cs.ias.wordnet.WordNet3;

public class MTurkParser implements IMTurkParser {

	/**
	 * 
	 * @param word word to highlight
	 * @param sentence in context
	 * @param tagger should be used to compare the index
	 * @return XHMTL formatted tag
	 */
	public String highlightWord(MTurkWord word, String sentence, IMTurkTagger tagger)
	{	
		String wordValue = word.getValue();
		int wordindex = word.getIndex();
		
		if(sentence.indexOf(wordValue) != sentence.lastIndexOf(wordValue))
		{
			//More than one occurency of the word
			int occurencies = 0;
			int myoccurency = 0;
			
			int index=0;
			try {
				for(String w : tagger.tokenize(sentence))
				{
					if(w.contains(wordValue))
						occurencies++;
					
					if(index == wordindex)
						myoccurency = occurencies;
						
					index++;
				}
				
				String result = "";
				for(int i = 1; i < myoccurency;i++)
				{
					int occindex = sentence.indexOf(wordValue);
					result += sentence.substring(0, occindex+wordValue.length());
					sentence = sentence.substring(occindex+wordValue.length());
				}
				
				result += sentence.replaceFirst(wordValue, "<font color='red'><b>"+wordValue+"</b></font>");
				
				return result;
			} catch (Exception e) {
				e.printStackTrace();
				return sentence.replaceAll(wordValue, "<font color='red'><b>"+wordValue+"</b></font>");
			}
		}else{
			return sentence.replace(wordValue, "<font color='red'><b>"+wordValue+"</b></font>");
		}
	}
	
	/**
	 * Creates an JSONObject of the Question Result
	 * 
	 * @param result MTurkQuestionResult
	 * 
	 * @return a json object that represents the result
	 * 
	 * @throws ParseException
	 */
	private JSONObject parseJSONResult(MTurkQuestionResult result) throws ParseException
	{		
		//Parses the questionIdentifier to JSON
		JSONObject root = new JSONObject(result.getQuestionIdentifier());
			
		//Parse answers
		JSONArray jsonAnswers = new JSONArray();
		for(String answer : result.getAnswers().keySet())
		{
			JSONObject answerObject = new JSONObject();
			answerObject.put("sense", answer);
			answerObject.put("votes", result.getAnswers().get(answer));
			jsonAnswers.put(answerObject);
		}	
		root.put("results", jsonAnswers);
			
		return root;
	}
	
	/**
	 * Returns the string representation of a POS-tag
	 * @param tag
	 * @return string repr.
	 */
	private String getReadableStringForTag(String tag)
	{
		POS pos = WordNet3.convertTagToWordnetPOS(tag);
		if(pos == POS.VERB)
			return "Verb";
		if(pos == POS.ADJECTIVE)
			return "Adjective";
		if(pos == POS.NOUN)
			return "Noun";
		if(pos == POS.ADVERB)
			return "Adverb";
		
		return "?";
	}
	
	/**
	 * Generates the questionIdentifier
	 * 
	 * @param question
	 * 
	 * @return string representation of the question
	 */
	public String serializeQuestion(MTurkQuestion question)
	{
		JSONObject j = new JSONObject();
		MTurkWord word = question.getWord();
		
		j.put("tag", word.getTag());
		j.put("word", word.getValue());
		j.put("index", word.getIndex());
		j.put("sentence", question.getSentence());
		j.put("context", question.getContext());
		
		if(question.getAnswer()!=null)
			j.put("answer", question.getAnswer());
		
		return j.toString();
	}
	
	/**
	 * Deserializes the questionIdentifier
	 * 
	 * @param s string representation
	 * 
	 * @return a MTurkQuestion
	 * @throws ParseException 
	 */
	public MTurkQuestion deserializeQuestion(String s) throws ParseException
	{
		JSONObject j = new JSONObject(s);
		
		String sentence = j.getString("sentence");
		String context = j.getString("context");
		
		int index = j.getInt("index");
		String value = j.getString("word");
		String tag = j.getString("tag");
		
		MTurkWord word = new MTurkWord(value,tag,index);
		
		MTurkQuestion question = new MTurkQuestion(word, sentence, context);
		
		if(j.has("answer"))
			question.setAnswer(j.getString("answer"));
		
		return question;
	}

	@Override
	/**
	 * Use Amazon Mechanical Turk AnswerKey XML Format
	 */
	public String parseAnswersKey(List<MTurkQuestion> questions) {
		
		String answerKey = "";
	    answerKey += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
	    answerKey += "<AnswerKey xmlns=\"http://mechanicalturk.amazonaws.com/AWSMechanicalTurkDataSchemas/2005-10-01/AnswerKey.xsd\">";
	    
	    int i = 0;
	    for(MTurkQuestion question : questions)
	    {
		    answerKey += "<Question>";
		    answerKey += "  <QuestionIdentifier>"+ serializeQuestion(questions.get(i)) +"</QuestionIdentifier>";
		    answerKey += "  <AnswerOption>";
		    answerKey += "    <SelectionIdentifier>"+question.getAnswer()+"</SelectionIdentifier>";
		    answerKey += "    <AnswerScore>"+1+"</AnswerScore>";
		    answerKey += "  </AnswerOption>";
		    answerKey += "</Question>";
		    
		    i++;
	    }
	    answerKey += "</AnswerKey>";
	    return answerKey;
	}

	@Override
	/**
	 * Use Amazon Mechanical Turk QuestionForm XML Format
	 */
	public String parseQuestionsForm(List<MTurkQuestion> questions,
			MTurkProperties properties, IMTurkTagger tagger) {
		
		String test = "";
		test += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
		test += "<QuestionForm xmlns=\"http://mechanicalturk.amazonaws.com/AWSMechanicalTurkDataSchemas/2005-10-01/QuestionForm.xsd\">";
		
		test += "<Overview>";
		
		test += "<Title>"+properties.getTitle()+"</Title>";
		test += "<FormattedContent><![CDATA[<br></br>]]></FormattedContent>";
		
		test += "<FormattedContent><![CDATA[<h3>Example 1</h3>]]></FormattedContent>";
		test += "<FormattedContent><![CDATA[<h2><font color='black'>Put the pans on the center <font color='red'><b>rack</b></font> for the preheated oven.</font></h2>]]></FormattedContent>";
		test +=	"<FormattedContent><![CDATA[Please select the most appropriate sense for the <b><font color='blue'>Noun</font> <font color='red'>rack</font></b> in the sentence above.]]></FormattedContent>";
		
		test += "<FormattedContent><![CDATA[" +
			"<ul>" +
			"<li><font color='black'>Wrong part of speech provided</font></li>" +
			"<li><font color='green'><b>framework for holding objects</b></font></li>" +
			"<li><font color='black'>rib section of a forequarter of veal or pork or especially lamb or mutton</font></li>" +
			"<li><font color='black'>the destruction or collapse of something</font> 'wrack and ruin'</li>" +
			"</ul>" +
			"]]></FormattedContent>";
		
		test += "<FormattedContent><![CDATA[<br></br>]]></FormattedContent>";
		
		test += "<FormattedContent><![CDATA[<h3>Example 2</h3>]]></FormattedContent>";
		test += "<FormattedContent><![CDATA[<h2><font color='black'>Put the <font color='red'><b>pans</b></font> on the center rack for the preheated oven.</font></h2>]]></FormattedContent>";
		test +=	"<FormattedContent><![CDATA[Please select the most appropriate sense for the <b><font color='blue'>Verb</font> <font color='red'>pans</font></b> in the sentence above.]]></FormattedContent>";
		test += "<FormattedContent><![CDATA[" +
			"<ul>" +
			"<li><font color='green'><b>Wrong part of speech provided</b></font></li>" +
			"<li><font color='black'>make a sweeping movement</font> 'The camera panned across the room'</li>" +
			"<li><font color='black'>express a totally negative opinion of</font> 'The critics panned the performance'</li>" +
			"</ul>" +
			"]]></FormattedContent>";
		
		test += "<FormattedContent><![CDATA[<br></br>]]></FormattedContent>";
		
		test += "<FormattedContent><![CDATA[<h3>NOTE:</h3>]]></FormattedContent>";
		test += "<FormattedContent><![CDATA[" +
				"<h5>If there is no appropriate sense in the presented list. You may choose one for the following options:</h5>" +
				"<ul>" +
				"<li><font color='black'>Select 'Wrong part of speech provided', for example if the word is a verb, but only noun senses available.</font></li>" +
				"<li><font color='black'>Select 'No matching sense found', if part of speech is correct, but no matching sense is listed.</font></li>" +
				"</ul>" +
				"]]></FormattedContent>";
		
		test += "</Overview>";
		test += "<Overview><FormattedContent><![CDATA[<h1>BEGINNING OF THE PROPER TASK</h1>]]></FormattedContent></Overview>";
		
		//Generate XML data for each question
		for(MTurkQuestion question:questions)
		{
			test += "<Question>";
			test += "  <QuestionIdentifier>"+ serializeQuestion(question) +"</QuestionIdentifier>";
			test += "  <DisplayName>Sentence: "+question.getSentence()+" Word: "+question.getWord().getValue()+"</DisplayName>";
			test += "  <IsRequired>true</IsRequired>";
			test += "  <QuestionContent>";
			test += "   <FormattedContent><![CDATA[<h2><font color='black'>"+highlightWord(question.getWord(),question.getSentence(),tagger)+"</font></h2>]]></FormattedContent>";
			test +=	"<FormattedContent><![CDATA[Please select the most appropriate sense for the <b><font color='blue'>"+getReadableStringForTag(question.getWord().getTag())+"</font> <font color='red'>"+question.getWord().getValue()+"</font></b> in the sentence above.]]></FormattedContent>";
			test += "  </QuestionContent>";
			test += "  <AnswerSpecification>";
			test += "   <SelectionAnswer>";
			test += "    <StyleSuggestion>radiobutton</StyleSuggestion>";
			test += "    <Selections>";

			//show wrong part of speech answer option
			if(properties.isOthertype())
			{
				test += "     <Selection>";
				test += "      <SelectionIdentifier>OTHERTYPE</SelectionIdentifier>";
				test += "		<FormattedContent><![CDATA[<font color='black'><b>Wrong part of speech provided</b></font>]]></FormattedContent>";
				test += "     </Selection>";
			}
			
			//Generate the possible answers
			int currentAnswer = 0;
			for (String display : question.getDisplayTexts()) 
			{
				//Check the limit
				if(properties.getLimit() >= 0 && currentAnswer >= properties.getLimit())
					break;
				
				String value = question.getValueTexts().get(currentAnswer);
				
				if(display.contains(";"))
					display = display.replaceFirst(";", "</b></font>");
				else 
					display = display + "</b></font>";
				
				display = "<font color='black'><b>"+ display;
				
				test += "     <Selection>";
				test += "      <SelectionIdentifier>" + value + "</SelectionIdentifier>";
				test += "		<FormattedContent><![CDATA["+display+"]]></FormattedContent>";
				test += "     </Selection>";
				
				currentAnswer++;
			}
			//Show no match answer option
			if(properties.isNomatch())
			{
				test += "     <Selection>";
				test += "      <SelectionIdentifier>NOMATCH</SelectionIdentifier>";
				test += "		<FormattedContent><![CDATA[<font color='black'><b>No matching sense found</b></font>]]></FormattedContent>";
				test += "     </Selection>";
			}
			test += "    </Selections>";
			test += "   </SelectionAnswer>";
			test += "  </AnswerSpecification>";
			test += "</Question>";
		}
		
		test += "</QuestionForm>";
		
		return test;
	}

	@Override
	/**
	 * Use JSON Output
	 */
	public String parseResults(List<MTurkResult> results) throws ParseException 
	{
		JSONArray root = new JSONArray();
		
		for(MTurkResult result : results)
			for(MTurkQuestionResult qResult : result.getResults())
			{
				root.put(parseJSONResult(qResult));
			}
		
		return root.toString();
	}

	@Override
	/**
	 * Converts a JSON string into a Workerpool-Object
	 */
	public HashMap<String,MTurkWorker> parseWorkerPool(String source) throws ParseException 
	{
		JSONArray workers = new JSONArray(source);
		
		HashMap<String,MTurkWorker> pool = new HashMap<String, MTurkWorker>();
		
		for(int i = 0; i < workers.length(); i++)
		{
			JSONObject workerRaw = workers.getJSONObject(i);
			
			String workerId = workerRaw.getString("workerId");
			
			MTurkWorker worker = new MTurkWorker(workerId);
			
			worker.setFalseAnswers(workerRaw.getInt("falseAnswers"));
			worker.setRightAnswers(workerRaw.getInt("rightAnswers"));
			worker.setBlocked(workerRaw.getBoolean("isBlocked"));
			pool.put(workerId, worker);
		}
		
		return pool;
	}

	@Override
	/**
	 * Converts Workerpool-object into JSON string
	 */
	public String parseWorkerPool(MTurkWorkerPool pool) 
	{
		JSONArray workers = new JSONArray();
		
		for(String workerId : pool.getWorkerIds())
		{
			MTurkWorker worker = pool.getWorker(workerId);
			JSONObject workerRaw = new JSONObject();
			workerRaw.put("workerId", worker.getWorkerId());
			workerRaw.put("falseAnswers", worker.getFalseAnswers());
			workerRaw.put("rightAnswers", worker.getRightAnswers());
			workerRaw.put("isBlocked", worker.isBlocked());
			workers.put(workerRaw);
		}
		
		return workers.toString();
	}
}
