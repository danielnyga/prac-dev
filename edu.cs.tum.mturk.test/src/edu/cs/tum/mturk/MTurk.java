package edu.cs.tum.mturk;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import com.amazonaws.mturk.addon.HITDataInput;
import com.amazonaws.mturk.addon.HITDataOutput;
import com.amazonaws.mturk.addon.HITProperties;
import com.amazonaws.mturk.dataschema.QuestionFormAnswersType;
import com.amazonaws.mturk.requester.Assignment;
import com.amazonaws.mturk.requester.HIT;
import com.amazonaws.mturk.requester.HITStatus;
import com.amazonaws.mturk.requester.QualificationType;
import com.amazonaws.mturk.requester.QualificationTypeStatus;
import com.amazonaws.mturk.service.axis.RequesterService;
import com.amazonaws.mturk.util.PropertiesClientConfig;

import edu.cs.tum.mturk.help.IMTurkParser;
import edu.cs.tum.mturk.help.IMTurkTagger;
import edu.cs.tum.mturk.help.MTurkHelper;
import edu.cs.tum.mturk.help.MTurkParser;
import edu.cs.tum.mturk.model.MTurkProperties;
import edu.cs.tum.mturk.model.MTurkQuestion;
import edu.cs.tum.mturk.model.MTurkResult;
import edu.cs.tum.mturk.model.MTurkWord;
import edu.cs.tum.mturk.model.MTurkWorker;
import edu.cs.tum.mturk.pos.StanfordPOSParser;
import edu.mit.jwi.item.ISynset;
import edu.mit.jwi.item.POS;

import edu.tum.cs.ias.wordnet.MySynsetID;
import edu.tum.cs.ias.wordnet.WordNet3;

/**
 * This class provides a simple interface to interact with Amazon Mechanical Turk
 * to annotate Word Sense Disambiguation Tasks
 * (post new tasks, evaluate submitted task, and more)
 * 
 * @author Stephan Epping
 *
 */
public class MTurk {

	public IMTurkTagger tagger = null;
	protected RequesterService service = null;
	protected IMTurkParser parser = null;

	protected boolean printInfos = false;
	
	/**
	 * Creates a new MTurk Service with a provided properties file
	 * @throws IOException
	 */
	public MTurk() throws IOException {
		super();

		// Create Service
		service = new RequesterService(new PropertiesClientConfig(
				"./etc/mturk.properties"));
		
		parser = new MTurkParser();
		
		tagger = new StanfordPOSParser();
	}
	
	public void sendNotification(String workerId, String message, String subject){
		service.notifyWorkers(subject, message, new String[]{workerId});
	}
	
	/**
	 * Creates an qualification test on MTurk
	 * 
	 * @param input test questions in a specific format (sentence	wordindex	synsetid)
	 * @param properties specifications similar to the standard HIT properties attributes (see sample file)
	 * @throws Exception
	 */
	public void createQualification(HITDataInput input, MTurkProperties properties) throws Exception
	{
		List<MTurkQuestion> secureContent = readSecureQuestions(input);
		
		//List of test question
		ArrayList<MTurkQuestion> questions = new ArrayList<MTurkQuestion>();
		
		addSecureContent(questions, secureContent, secureContent.size(), false);
		
		//Parse List of questions to MTurk XML Question Format
		String questionXML = parser.parseQuestionsForm(questions, properties,tagger);
		
		//Parse List of answers to MTurk AnswerKey
		String answerXML = parser.parseAnswersKey(questions);
		
		//Post task online (use properties for title etc.)
		QualificationType qualification = service.createQualificationType(
				properties.getTitle(), 
				properties.getKeywords(), 
				properties.getDescription(), 
				QualificationTypeStatus.Active, 
				properties.getLifetime(), 
				questionXML, 
				answerXML, 
				properties.getAssignmentDuration(), 
				null, 
				null);
	
		//Print QualificationId for qualifications requirements use
		System.out.println("Created Qualification:"+qualification.getName()+" id:"+qualification.getQualificationTypeId()+" question:"+questions.size());
	}
	
	/**
	 * Expire the specified tasks now
	 * 
	 * @param input list of HITs to expire
	 */
	public void expireTasks(HITDataInput input)
	{
		//Check input format
		if(!MTurkHelper.validateInputForTasks(input))
			throw new IllegalArgumentException("Wrong input format for Tasks");
		
		//Proccess each task
		for(int i=1; i < input.getNumRows(); i++)
		{
			Map<String,String> row = input.getRowAsMap(i);
			String hitId = row.get(HITProperties.HITField.HitId.getFieldName());
			
			HIT hit = service.getHIT(hitId);
			
			//Check HIT state
			if(hit.getHITStatus() == HITStatus.Assignable 
			|| hit.getHITStatus() == HITStatus.Unassignable)
			{
				//Expire HIT
				service.forceExpireHIT(hitId);
				System.out.println("Expired Task:"+hitId+" "+hit.getTitle()+" "+hit.getDescription());
			}
		}
	}
	
	/**
	 * Extends the specified tasks for a certain time and number of assignments
	 * 
	 * @param input List of HITs
	 * @param assignmentsIncrement minimum number is 1
	 * @param expirationIncrementInSeconds time to extend on each task
	 */
	public void extendTasks(HITDataInput input, Integer assignmentsIncrement, Long expirationIncrementInSeconds)
	{
		//Check input format
		if(!MTurkHelper.validateInputForTasks(input))
			throw new IllegalArgumentException("Wrong input format for Tasks");
		
		//Process each task
		for(int i=1; i < input.getNumRows(); i++)
		{
			Map<String,String> row = input.getRowAsMap(i);
			String hitId = row.get(HITProperties.HITField.HitId.getFieldName());
			//HIT hit = service.getHIT(hitId);
			//Extend task
			service.extendHIT(hitId, assignmentsIncrement, expirationIncrementInSeconds);
			System.out.println("Extended Task:"+hitId);
			//System.out.println("Extended Task:"+hitId+" "+hit.getTitle()+" "+hit.getDescription());
		}
	}
	
	/**
	 * Evaluate a specific task
	 * 
	 * @param hitId unique task id
	 * @param output location, if task can NOT be evaluated completely
	 *  
	 * @return the result, which can be used to commit(pay the worker) and export all results
	 * 
	 * @throws IOException
	 */
	public MTurkResult evaluateTask(String hitId, HITDataOutput output, MTurkWorkerPool pool) throws Exception
	{
		boolean live = true;
		
		//Download HIT information
		HIT hit = service.getHIT(hitId);
		
		System.out.println(hit.getHITStatus());
		//HIT is ready
		int approvableAssignments = hit.getMaxAssignments() - hit.getNumberOfAssignmentsAvailable() - hit.getNumberOfAssignmentsPending() - hit.getNumberOfAssignmentsCompleted();
		//int doneAssignments = hit.getMaxAssignments() - hit.getNumberOfAssignmentsAvailable() - hit.getNumberOfAssignmentsPending();
		
		if(approvableAssignments > 0)
		{	
			//Approve the assignments
			for(Assignment a : service.getAllSubmittedAssignmentsForHIT(hit.getHITId()))
			{
				MTurkWorker worker = pool.getWorker(a.getWorkerId());
				
				boolean block = false;
				
				if(live)
					block = updateWorker(a, worker);
				
				if(!worker.isBlocked() && block)
				{
					if(live)
					{
						service.blockWorker(worker.getWorkerId(), "Sorry, too many wrong answered security question."+" (wrong:"+worker.getFalseAnswers()+" / right:"+worker.getRightAnswers()+")");
						worker.setBlocked(true);
					}
					System.out.println("!-->Blocked Worder: "+worker.getWorkerId()+" (wrong:"+worker.getFalseAnswers()+" / right:"+worker.getRightAnswers()+")");
				}
				
				if(!worker.isBlocked())
				{
					System.out.println("->Approve "+hit.getHITId()+" for worker "+a.getWorkerId());
					if(live)
						service.approveAssignment(a.getAssignmentId(), "Thank you very much for working on our tasks. Your IAS team.");
				}else{
					System.out.println("!->Rejected "+hit.getHITId()+" for worker "+a.getWorkerId());
					if(live)
						service.rejectAssignment(a.getAssignmentId(), "Sorry, too many wrong answered security question."+" (wrong:"+worker.getFalseAnswers()+" / right:"+worker.getRightAnswers()+"). Contact for further information.");
				}
			}
		}
		
		//Download all assignments
		Assignment[] assignments = service.getAllAssignmentsForHIT(hit.getHITId());
		
		if(assignments.length < hit.getMaxAssignments())
		{
			if(hit.getHITStatus() == HITStatus.Reviewable)
			{
				//Not reviewable yet, wait for more assignments or until expired
				MTurkHelper.writeInfos(output, hit, "Expired,but not completed.("+assignments.length+"/"+hit.getMaxAssignments()+")");
				System.out.println("Hit:"+hit.getHITId()+ " - Expired,but not completed.("+assignments.length+"/"+hit.getMaxAssignments()+")");
			}else{
				//Not reviewable yet, wait for more assignments or until expired
				MTurkHelper.writeInfos(output, hit, "Not all assignments done yet.("+assignments.length+"/"+hit.getMaxAssignments()+")");
				System.out.println("Hit:"+hit.getHITId()+ " - Not all assignments done yet.("+assignments.length+"/"+hit.getMaxAssignments()+")");
			}
		}
		MTurkResult result = null;
		
		if(assignments.length > 0)
			result =  new MTurkResult(hit, assignments,parser,pool);
		
		if(result!=null)
			System.out.println("Created results: "+hit.getHITId()+" Assignments:"+result.getAssignments().length);
		
		return result;
	}
	
	/**
	 * Evaluates all tasks in the input file
	 * 
	 * @param input List of HITs
	 * @param output Not processed tasks are stored here
	 * 
	 * @return List of Task results
	 * 
	 * @throws IOException
	 */
	public ArrayList<MTurkResult> evaluateTasks(HITDataInput input, HITDataOutput output, MTurkWorkerPool pool) throws Exception
	{	
		//Check input format
		if(!MTurkHelper.validateInputForTasks(input))
			throw new IllegalArgumentException("Wrong input format for Tasks");
		
		ArrayList<MTurkResult> results = new ArrayList<MTurkResult>();
		
		//Process each task
		for(int i=1; i < input.getNumRows(); i++)
		{
			Map<String,String> row = input.getRowAsMap(i);
			String hitId = row.get(HITProperties.HITField.HitId.getFieldName());
			
			//Evaluare task
			MTurkResult result = evaluateTask(hitId, output, pool);
			
			//Add to overall results
			if(result != null)
				results.add(result);
		}
		
		return results;	
	}
	
	/**
	 * Adds secure, verifyable questions
	 * 
	 * @param questions to this questions the verifyable ones should be added
	 * @param secureContent List of secure questions
	 * @param secureQuestions number of secure questions, which should be added
	 * @param random if true, the secure questions will be randomly added
	 * @throws Exception
	 */
	protected void addSecureContent(List<MTurkQuestion> questions, List<MTurkQuestion> secureContent,
			int secureQuestions, boolean random) throws Exception
	{
		if(secureContent == null)
			return;
		
		Random r = new Random();
		
		for(int i=0; i<secureQuestions;i++)
		{	
			int pos = questions.size();
			int secPos = i;
			
			if(random)
			{
				//Randomly select question and position in task
				pos = r.nextInt(questions.size()+1);
				secPos = r.nextInt(secureContent.size());
			}
			
			MTurkQuestion q = secureContent.get(secPos);

			questions.add(pos, q);
			
			System.out.println(" Added Secure Question: "+q.getWord().getValue()+" "+q.getWord().getTag()+" "+q.getSentence());
		}
	}

	/**
	 * Creates task with some secure questions inside
	 * 
	 * @param sentences List of sentences to process
	 * @param properties information about the task like title, assignments, etc
	 * @param hitTypeId if null an new task group will be created
	 * @param logfile location, where all posted task are stored
	 * @param acc number of questions in each task
	 * @param input inputfile with verifyable questions
	 * @param secureQuestions number of secure questions per task
	 * 
	 * @return number of posted tasks
	 * 
	 * @throws Exception
	 */
	public int createSecureTasks(String[] sentences,String context, MTurkProperties properties, String hitTypeId, HITDataOutput logfile, int acc, HITDataInput input, int secureQuestions) throws Exception 
	{
		List<MTurkQuestion> questions = new ArrayList<MTurkQuestion>();
		List<MTurkQuestion> secureContent = readSecureQuestions(input);
		
		int accCounter = 0;
		
		int numberOfTasks = 0;
		
		int index = 0;
		for (String sentence : sentences) 
		{
			// Tag words with stanford POS Tagger
			ArrayList<MTurkWord> taggedWords = tagger.tag(sentence);
			
			// Resolve words with wordnet
			HashMap<MTurkWord, List<ISynset>> resolvedWords = getWordNetInfos(taggedWords);

			int index1 = 0;
			for (MTurkWord word : resolvedWords.keySet()) 
			{
				accCounter++;
				
				//Generate MTurkQuestion
				//TODO check if synsets > 0 ?
				questions.add(generateQuestion(word, sentence, resolvedWords.get(word), index, context));
				index1++;
				
				if(accCounter == acc)
				{	
					System.out.println(" Done "+index1+" of Sentence: "+sentence);
					addSecureContent(questions,secureContent,secureQuestions,true);
					//Enough questions collected, post new task
					HIT hit = postQuestions(questions, hitTypeId, properties);

					//Log task infos
					MTurkHelper.writeInfos(logfile, hit, "");
					accCounter = 0;
					questions.clear();
					
					numberOfTasks++;
					index1 = 0;
				}
			}
			
			if(index1 > 0)
				System.out.println(" Done "+index1+" of Sentence: "+sentence);
			
			index++;
		}
		
		//Do remaining questions
		if(questions.size() > 0)
		{
			addSecureContent(questions,secureContent,secureQuestions,true);
			//post task
			HIT hit = postQuestions(questions, hitTypeId, properties);

			//Log task infos
			MTurkHelper.writeInfos(logfile, hit, "");
			numberOfTasks++;
		}
		
		return numberOfTasks;
	}
	
	/**
	 * Creates a new tasks for every word (Noun, Verbs, Adverb, Adjectives) in the provided sentences
	 * 
	 * @param sentences List of sentences to process
	 * @param properties information about the task like title, assignments, etc
	 * @param hitTypeId if null an new task group will be created
	 * @param logfile location, where all posted task are stored
	 * @param acc number of questions in each task
	 * 
	 * @return number of posted tasks
	 * 
	 * @throws Exception
	 */
	public int createTasks(String[] sentences, String context,MTurkProperties properties, String hitTypeId, HITDataOutput logfile, int acc) throws Exception 
	{
		return createSecureTasks(sentences, context, properties, hitTypeId, logfile, acc, null, 0);
	}

	/**
	 * Creates a new task group
	 * 
	 * @param properties properties to identify the group
	 * 
	 * @return new taskgroup identifier
	 */
	public String createTaskGroup(HITProperties properties) 
	{
		//Ask MTurk for a new taskgroup identifier
		String hitTypeId = service.registerHITType(properties.getAutoApprovalDelay(),
				properties.getAssignmentDuration(), properties
						.getRewardAmount(), properties.getTitle(), properties
						.getKeywords(), properties.getDescription(), properties
						.getQualificationRequirements());
		
		System.out.println("Created hitTypeId: "+hitTypeId);
		return hitTypeId;
	}

	/**
	 * Post a new Task
	 * 
	 * @param questions List of questions to annotate in this task
	 * @param hitTypeId specific hitgroup, creates new or reuses one
	 * @param properties information about the task like title, assignments, etc
	 * 
	 * @return a new HIT
	 */
	protected HIT postQuestions(List<MTurkQuestion> questions, String hitTypeId,
			MTurkProperties properties) {
		
		HIT hit = service.createHIT(hitTypeId, // hitTypeId
				properties.getTitle(), // title
				properties.getDescription(), // description
				properties.getKeywords(), // keywords
				parser.parseQuestionsForm(questions,properties,tagger), // XML format of question
				properties.getRewardAmount(), // reward
				properties.getAssignmentDuration(), // assignmentDurationInSeconds
				properties.getAutoApprovalDelay(), // autoApprovalDelayInSeconds
				properties.getLifetime(), // lifetime
				properties.getMaxAssignments(), // maxassignments
				null, // annotation
				properties.getQualificationRequirements(), // qualification requirements
				null // response group
				);
		
		System.out.println("Created HIT:"+hit.getHITId()+" "+hit.getTitle()+" "+hit.getDescription()+" questions:"+questions.size());
		return hit;
	}

	/**
	 * Generate a new MTurkQuestion
	 * 
	 * @param word TaggedWord to annotate
	 * @param sentence context of the specified word
	 * @param synsets possible synsets of the specified word
	 * @param context 
	 * @param index 
	 * 
	 * @return a new MTurkQuestion
	 */
	protected MTurkQuestion generateQuestion(MTurkWord word, String sentence, List<ISynset> synsets, int fileindex, String context) {
		
		context = context+";line "+fileindex;
		//Create new question
		MTurkQuestion question = new MTurkQuestion(word, sentence, context);

		//Serialized List of SynsetIDs
		ArrayList<String> values = new ArrayList<String>();
		
		//List of displayed information for each synset
		ArrayList<String> display = new ArrayList<String>();

		for (ISynset s : synsets) 
		{
			//Create serializable datastructure
			MySynsetID id = new MySynsetID(s.getOffset(), s.getPOS());

			display.add(s.getGloss());
			values.add(id.toString());
		}

		//Add options to MTurkQuestion
		question.setValueTexts(values);
		question.setDisplayTexts(display);

		return question;
	}

	/**
	 * Collect synsets for each word (POS: Noun, Verb, Adverb, Adjective)
	 * Following words of the same POS will be concatenated,
	 * if the new word has a minimum of one synset provided by WordNet
	 * 
	 * @param taggedWords words to tag (filtered by POS)
	 * 
	 * @return Collection of TaggedWords and associated wordsenses(synsets)
	 */
	protected HashMap<MTurkWord, List<ISynset>> getWordNetInfos(ArrayList<MTurkWord> taggedWords) 
	{
		HashMap<MTurkWord, List<ISynset>> result = new HashMap<MTurkWord, List<ISynset>>();

		MTurkWord lastWord = null;
		
		//Process each word
		for (MTurkWord word : taggedWords) 
		{
			//Convert StanfordTagger POS to WordNet POS
			
			if(lastWord != null)
			{
				//Convert POS
				POS currenttype = WordNet3.convertTagToWordnetPOS(word.getTag());
				POS lasttype = WordNet3.convertTagToWordnetPOS(lastWord.getTag());
				
				//Check if the last and current word have the same POS
				if(lasttype == POS.NOUN || currenttype == POS.NOUN)
				{
					//Concatenate words
					String connectedWord = lastWord.getValue() + " " + word.getValue();
					//Check wordnet
					List<ISynset> wordSynSet = WordNet3.getSynsets(connectedWord,
							POS.NOUN);
					
					//If connectedWord has own synsets at WordNet, use it instead of separate
					if(wordSynSet.size() > 0)
					{
						//Take this one, remove old
						result.remove(lastWord);
			
						if (printInfos)
							System.out.println("! Wordnet removed '" + lastWord.getValue() +"' and added '"+connectedWord+"' instead!");
						
						//Update Word
						word = new MTurkWord(connectedWord, WordNet3.PENN_NN,lastWord.getIndex());
					}
				}
			}
			POS type = WordNet3.convertTagToWordnetPOS(word.getTag());
			//Relevant part of speech
			if (type != null) 
			{
				//Collect Synsets
				List<ISynset> wordSynSet = WordNet3.getSynsets(word.getValue(),
						type);

				if (printInfos)
					System.out.println("Wordnet resolved '" + word.getValue()
							+ "' as " + type + ": " + wordSynSet);

				//save synset for target word
				result.put(word, wordSynSet);
			}
			
			lastWord = word;
		}

		return result;
	}
	
	/**
	 * Reads a file of qualification/secure questions
	 * 
	 * @param input file containing the qualification sentences
	 * @return a list of qualification questions
	 * @throws Exception
	 */
	protected List<MTurkQuestion> readSecureQuestions(HITDataInput input) throws Exception
	{
		if(input == null)
			return null;
		
		//Check input format
		if(!MTurkHelper.validateInputForQualification(input))
			throw new IllegalArgumentException("Wrong input format for secure questions");
		
		List<MTurkQuestion> secureQuestions = new ArrayList<MTurkQuestion>();
		
		//First row is headerrow
		for(int i = 1; i < input.getNumRows(); i++)
		{
			Map<String, String> values = input.getRowAsMap(i);
			
			int index = Integer.parseInt(values.get("word"));
			String sentence = values.get("sentence");
			String answer = values.get("answer");
			
			//Run POS tagger
			ArrayList<MTurkWord> taggedWords = tagger.tag(sentence);
			
			// Resolve words with wordnet
			HashMap<MTurkWord, List<ISynset>> resolvedWords = getWordNetInfos(taggedWords);

			MTurkWord word = taggedWords.get(index);
			//Generate MTurkQuestion for this word
			MTurkQuestion question = generateQuestion(word, sentence, resolvedWords.get(word),i,"qualification");
			question.setAnswer(answer);
			
			secureQuestions.add(question);
		}
		
		return secureQuestions;
	}
	
	/**
	 * Updates the statistics of a worker in the given workerpool
	 * @param assignment current assigment, which is used to update the worker
	 * @param worker the concrete worker submitted the assignment
	 * @return true, if the worker should be blocked, because of his bad statistics
	 * @throws Exception
	 */
	protected boolean updateWorker(Assignment assignment, MTurkWorker worker) throws Exception
	{	
		for (QuestionFormAnswersType.AnswerType answer : MTurkResult.getAnswers(assignment)) 
		{
			String hisAnswer = RequesterService.getAnswerValue(assignment.getAssignmentId(), answer);
		
			MTurkQuestion question = parser.deserializeQuestion(answer.getQuestionIdentifier());
			
			//Its a secure question
			if(question.getAnswer()!=null)
			{
				String truthAnswer = question.getAnswer();
				
				if(hisAnswer.equals(truthAnswer))
					worker.incrementRight();
				else{
					System.out.println("!--> Secure:"+truthAnswer+" - "+worker.getWorkerId()+" said "+hisAnswer);
					worker.incrementFalse();
				}
			}	
		}
		
		double right = (double) worker.getRightAnswers();
		double notright = (double) worker.getFalseAnswers();
		
		//Worker should be blocked?
		//TODO variable values
		if(right+notright > 30 && notright/(right + notright) > 0.35)
			return true;
		
		return false;
	}
	
	public IMTurkParser getTurkParser()
	{
		return parser;
	}
	
	public void setTurkParser(IMTurkParser p)
	{
		this.parser = p;
	}

	public IMTurkTagger getTagger() {
		return tagger;
	}

	public void setTagger(IMTurkTagger tagger) {
		this.tagger = tagger;
	}
}
