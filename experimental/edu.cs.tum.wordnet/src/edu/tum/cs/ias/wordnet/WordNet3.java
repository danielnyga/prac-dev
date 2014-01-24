package edu.tum.cs.ias.wordnet;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import edu.mit.jwi.Dictionary;
import edu.mit.jwi.IDictionary;
import edu.mit.jwi.item.IIndexWord;
import edu.mit.jwi.item.ISenseKey;
import edu.mit.jwi.item.ISynset;
import edu.mit.jwi.item.ISynsetID;
import edu.mit.jwi.item.IWord;
import edu.mit.jwi.item.IWordID;
import edu.mit.jwi.item.POS;
import edu.mit.jwi.item.Pointer;
import edu.mit.jwi.item.SenseKey;
import edu.mit.jwi.morph.IStemmer;
import edu.mit.jwi.morph.SimpleStemmer;
import edu.mit.jwi.morph.WordnetStemmer;

/**
 * 
 * Wrapper class providing some convenience methods of the Stanford WordNet Java
 * interface.
 * 
 * @author Daniel Nyga
 * 
 */
public class WordNet3 {

	/** Coordinating Conjunction */
	public static final String PENN_CC = "CC";
	/** Cardinal Number */
	public static final String PENN_CD = "CD";
	/** Determiner */
	public static final String PENN_DT = "DT";
	/** Existential "there" */
	public static final String PENN_EX = "EX";
	/** Foreign Word */
	public static final String PENN_FW = "FW";
	/** Preposition or subordinating conjunction */
	public static final String PENN_IN = "IN";
	/** Adjective */
	public static final String PENN_JJ = "JJ";
	/** Adjective, comparative */
	public static final String PENN_JJR = "JJR";
	/** Adjective, superlative */
	public static final String PENN_JJS = "JJS";
	/** List item marker */
	public static final String PENN_LS = "LS";
	/** Modal */
	public static final String PENN_MD = "MD";
	/** Noun, singular or mass */
	public static final String PENN_NN = "NN";
	/** Noun, plural */
	public static final String PENN_NNS = "NNS";
	/** Proper noun */
	public static final String PENN_NNP = "NNP";
	/** Proper noun, plural */
	public static final String PENN_NNPS = "NNPS";
	/** Predeterminer */
	public static final String PENN_PDT = "PDT";
	/** Possessive ending */
	public static final String PENN_POS = "POS";
	/** Personal pronoun */
	public static final String PENN_PRP = "PRP";
	/** Possessive pronoun */
	public static final String PENN_PRP$ = "PRP$";
	/** Adverb */
	public static final String PENN_RB = "RB";
	/** Adverb, comparative */
	public static final String PENN_RBR = "RBR";
	/** Adverb, superlative */
	public static final String PENN_RBS = "RBS";
	/** Particle */
	public static final String PENN_RP = "RP";
	/** Symbol */
	public static final String PENN_SYM = "SYM";
	/** "to" */
	public static final String PENN_TO = "TO";
	/** Interjection */
	public static final String PENN_UH = "UH";
	/** Verb, base form */
	public static final String PENN_VB = "VB";
	/** Verb, past tense */
	public static final String PENN_VBD = "VBD";
	/** Verb, gerund or present participle */
	public static final String PENN_VBG = "VBG";
	/** Verb, past participle */
	public static final String PENN_VBN = "VBN";
	/** Verb, non-3rd person singular present */
	public static final String PENN_VBP = "VBP";
	/** Verb, 3-rd person singular present */
	public static final String PENN_VBZ = "VBZ";
	/** Wh-determiner */
	public static final String PENN_WDT = "WDT";
	/** Wh-pronoun */
	public static final String PENN_WP = "WP";
	/** Possessive wh-pronoun */
	public static final String PENN_WP$ = "WP$";
	/** Wh-adverb */
	public static final String PENN_WRB = "WRB";

	private static IDictionary dict = null;

	private static String wnhome = "";

	/** Maps from an irregular verb form -> base form */
	protected static HashMap<String, String> irregularVerbs = null;

	/** Maps from irregular noun forms -> base form */
	protected static HashMap<String, String> irregularNouns = null;

	/** Maps from adjective form -> base form */
	protected static HashMap<String, String> irregularAdj = null;

	/** Maps from adverb form -> base form */
	protected static HashMap<String, String> irregularAdv = null;

	protected static HashMap<ISynsetID, Integer> subtreeHeights = null;

	static {
		init();
	}

	public static ISynset unitOfMeasureSynset = WordNet3
			.getSynsetFromID(new MySynsetID(33615, POS.NOUN));

	public static ISynset unitOfTimeSynset = WordNet3
			.getSynsetFromID(new MySynsetID(15154774, POS.NOUN));

	public WordNet3() {
		
	}
	
	/**
	 * 
	 */
	private static void init() {
		try {
			wnhome = System.getenv("WNHOME");

			if (wnhome == null)
				throw new Exception("WNHOME variable not set");

			String path = wnhome + File.separator + "dict";
			URL url = new URL("file", null, path);

			dict = new Dictionary(url);
			dict.open();

			// Cache irregular words
			cacheIrregularNouns();
			cacheIrregularVerbs();
			cacheIrregularAdjectives();
			cacheIrregularAdverbs();

		} catch (Exception e) {
			System.err.println("[WordNet]: ERROR: Cannot initalize WordNet:");
			e.printStackTrace();
		}
	}

	public static boolean existsAsIrregularVerb(String word) {
		return irregularVerbs.containsKey(word.toLowerCase());
	}

	/**
	 * Converts the Stanford POS tag to Wordnet POS tag
	 * 
	 * @param tag
	 *            to convert
	 * 
	 * @return null, if no conversion possible
	 */
	public static POS convertTagToWordnetPOS(String tag) {
		/* Noun */
		if (tag.equals(WordNet3.PENN_NN) || tag.equals(WordNet3.PENN_NNP)
				|| tag.equals(WordNet3.PENN_NNPS)
				|| tag.equals(WordNet3.PENN_NNS))
			return POS.NOUN;

		/* Verb */
		if (tag.equals(WordNet3.PENN_VB) || tag.equals(WordNet3.PENN_VBD)
				|| tag.equals(WordNet3.PENN_VBG)
				|| tag.equals(WordNet3.PENN_VBN)
				|| tag.equals(WordNet3.PENN_VBP)
				|| tag.equals(WordNet3.PENN_VBZ))
			return POS.VERB;

		/* Adverb */
		if (tag.equals(WordNet3.PENN_RB) || tag.equals(WordNet3.PENN_RBR)
				|| tag.equals(WordNet3.PENN_RBS))
			return POS.ADVERB;

		/* Adjective */
		if (tag.equals(WordNet3.PENN_JJ) || tag.equals(WordNet3.PENN_JJR)
				|| tag.equals(WordNet3.PENN_JJS))
			return POS.ADJECTIVE;

		return null;
	}

	/**
	 * Returns a list of synsets the given word belongs to, given a specific
	 * part of speech
	 * 
	 * @param word
	 * @param pos
	 * @return
	 */
	public static List<ISynset> getSynsets(String word, POS pos) {

		List<ISynset> result = new ArrayList<ISynset>();

		if (word == null || pos == null)
			return result;

		IIndexWord idxword = dict.getIndexWord(word, pos);

		// If not word indices can be found, maybe the word has a particular
		// form so try to get its word stem and retry
		String stem = getWordStem(word, pos);

		if (idxword == null && stem != null) {
			idxword = dict.getIndexWord(stem, pos);
		}

		if (idxword == null)
			return result;

		List<IWordID> wordIDs = idxword.getWordIDs();

		for (Iterator<IWordID> i = wordIDs.iterator(); i.hasNext();) {
			IWordID wordID = i.next();
			IWord iword = dict.getWord(wordID);
			result.add(iword.getSynset());
		}

		return result;
	}

	/**
	 * Returns the WordNet sense keys for a given word and POS
	 * 
	 * @param word
	 * @param pos
	 * @return
	 */
	public static List<ISenseKey> getSenseKeys(String word, POS pos) {

		List<ISenseKey> keys = new ArrayList<ISenseKey>();
		List<ISynset> synsets = getSynsets(word, pos);
		String lemma = getWordStem(word, pos);

		for (ISynset synset : synsets) {
			IWord iword = null;
			for (IWord w : synset.getWords()) {
				if (w.getLemma().replaceAll("_", " ").equalsIgnoreCase(lemma)) {
					iword = w;
					break;
				}
			}
			keys.add(iword.getSenseKey());
		}
		return keys;
	}

	/**
	 * Checks whether the given word exists in WordNet data base, considering
	 * different word forms, e.g. plural forms for nouns or particular verb
	 * forms.
	 * 
	 * @see WordNet.wordExactlyExistsAs(String, int)
	 * 
	 * @param word
	 * @param type
	 * @return
	 * @throws Exception
	 * @throws MalformedURLException
	 */
	public static boolean wordExistsAs(String word, POS pos) {

		boolean found = wordExactlyExistsAs(word, pos);

		if (!found) {
			String stem = getWordStem(word, pos);
			found = wordExactlyExistsAs(stem, pos);
		}

		return found;
	}

	/**
	 * Checks whether the given word exists in WordNet data base, not
	 * considering different word forms.
	 * 
	 * @see WordNet.wordExistsAs(String, pos)
	 * 
	 * @param word
	 * @param pos
	 * @return
	 * @throws Exception
	 */
	public static boolean wordExactlyExistsAs(String word, POS pos) {

		if (word == null)
			return false;

		return dict.getIndexWord(word, pos) != null;
	}

	public static ISynset getSynsetFromID(ISynsetID id) {
		return dict.getSynset(id);
	}

	/**
	 * Returns the List of synset ids that are possible for the given word and
	 * pos.
	 * 
	 * @param word
	 * @param pos
	 * @return
	 */
	public static List<ISynsetID> getSynsetIDs(String word, POS pos) {
		List<ISynset> synsets = getSynsets(word, pos);
		List<ISynsetID> synsetids = new ArrayList<ISynsetID>();
		for (ISynset synset : synsets)
			synsetids.add(synset.getID());
		return makeSerializable(synsetids);
	}

	public static boolean isUnitOfMeasure(String word) {
		List<ISynset> synsets = WordNet3.getSynsets(word, POS.NOUN);

		for (ISynset synset : synsets) {
			List<List<ISynsetID>> paths = WordNet3.hypernymsPaths(synset
					.getID());
			for (List<ISynsetID> path : paths)
				if (path.contains(unitOfMeasureSynset.getID()))
					return true;
		}

		return false;
	}

	public static boolean isUnitOfTime(String word) {
		List<ISynset> synsets = WordNet3.getSynsets(word, POS.NOUN);

		for (ISynset synset : synsets) {
			List<List<ISynsetID>> paths = WordNet3.hypernymsPaths(synset
					.getID());
			for (List<ISynsetID> path : paths)
				if (path.contains(unitOfTimeSynset.getID()))
					return true;
		}

		return false;

	}

	/**
	 * Returns the stem of the word <code>word</code> postulating the
	 * part-of-speech <code>pos</code>.
	 */
	public static String getWordStem(String word, POS pos) {

		if (word == null || pos == null)
			return null;

		word = word.toLowerCase();

		String stem = null;
		boolean found = false;

		if (pos == POS.VERB) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);

			if (!found) {
				stem = irregularVerbs.get(stem);
				found = wordExactlyExistsAs(stem, pos);
			}

			if (!found && word.endsWith("ed")) {
				stem = word.substring(0, word.length() - 1);
				found = wordExactlyExistsAs(stem, pos);
				if (!found) {
					stem = word.substring(0, word.length() - 2);
					found = wordExactlyExistsAs(stem, pos);
				}
			}

			if (!found && word.endsWith("ing")) {
				stem = word.substring(0, word.length() - 3);
				found = wordExactlyExistsAs(stem, pos);
				if (!found) {
					stem += "e";
					found = wordExactlyExistsAs(stem, pos);
				}

				if (!found) {
					stem = irregularVerbs.get(stem);
					found = wordExactlyExistsAs(stem, pos);
				}
			}

			if (!found && word.endsWith("s")) {
				stem = word.substring(0, word.length() - 1);
				found = wordExactlyExistsAs(stem, pos);
			}
		}

		else if (pos == POS.ADVERB) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);

			if (!found) {
				stem = irregularAdv.get(stem);
				found = wordExactlyExistsAs(stem, pos);
			}
		}

		else if (pos == POS.ADJECTIVE) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);

			if (!found) {
				if (word.endsWith("er") || word.endsWith("est")) {
					stem = word.substring(0, word.length() - 1);
					found = wordExactlyExistsAs(stem, pos);
					if (!found) {
						stem = word.substring(0, word.length() - 2);
						found = wordExactlyExistsAs(stem, pos);
						if (!found) {
							stem = word.substring(0, word.length() - 3);
							found = wordExactlyExistsAs(stem, pos);
							if (!found) {
								stem = irregularAdj.get(word);
								found = wordExactlyExistsAs(stem, pos);
							}
						}
					}
				}
			}
		}

		else if (pos == POS.NOUN) {
			stem = word;
			found = wordExactlyExistsAs(stem, pos);
			if (!found) {
				if (word.endsWith("s")) {
					stem = word.substring(0, word.length() - 1);
					found = wordExactlyExistsAs(stem, pos);
				}
				if (!found && word.endsWith("es")) {
					stem = word.substring(0, word.length() - 2);
					found = wordExactlyExistsAs(stem, pos);
				}
				if (!found) {
					if (irregularNouns.get(word) != null) {
						stem = irregularNouns.get(word);
						found = wordExactlyExistsAs(stem, pos);
					}
				}
			}
		}

		if (found)
			return stem;
		else
			return null;
	}

	/**
	 * Converts our part of speech types to the JWI POS classes
	 * 
	 * @param type
	 * @return
	 */
	// public static POS convertPOS(int type) {
	//
	// POS pos = null;
	//
	// if (type == Word.TYPE_VERB_INFINITIVE
	// || type == Word.TYPE_PAST_PARTICIPLE
	// || type == Word.TYPE_GERUND)
	// pos = POS.VERB;
	// else if (type == Word.TYPE_NOUN)
	// pos = POS.NOUN;
	// else if (type == Word.TYPE_ADV)
	// pos = POS.ADVERB;
	// else if (type == Word.TYPE_ADJ)
	// pos = POS.ADJECTIVE;
	//
	// return pos;
	// }

	public static void printPath(ISynsetID[] path) {
		for (int i = 0; i < path.length; i++) {
			System.out.print(path[i]);
			if (i < path.length - 1)
				System.out.print("->");
		}
		System.out.println();
	}

	/**
	 * Returns the set of <it>direct</it> hypernyms of the given synsetID.
	 * 
	 * @param synsetID
	 * @return
	 */
	public static List<ISynsetID> getHypernyms(ISynsetID synsetID) {
		ISynset syn = dict.getSynset(synsetID);

		if (syn == null)
			return new ArrayList<ISynsetID>();

		List<ISynsetID> hypernyms = syn.getRelatedSynsets(Pointer.HYPERNYM);

		return makeSerializable(hypernyms);
	}

	/**
	 * Converts the SynsetID instances to the serializable variant MySynsetID.
	 * 
	 * @param synset
	 * @return
	 */
	public static ISynsetID makeSerializable(ISynsetID synset) {
		MySynsetID newSynset = new MySynsetID(synset.getOffset(),
				synset.getPOS());
		return newSynset;
	}

	/**
	 * Converts the SynsetIDs in the list to serialize variants MySynset.
	 * 
	 * @param synsets
	 * @return
	 */
	public static List<ISynsetID> makeSerializable(List<ISynsetID> synsets) {
		List<ISynsetID> newSynsets = new ArrayList<ISynsetID>();
		for (Iterator<ISynsetID> i = synsets.iterator(); i.hasNext();) {
			newSynsets.add(makeSerializable(i.next()));
		}
		return newSynsets;
	}

	/**
	 * Returns the set of <it>direct</it> hyponyms of the given synsetID.
	 * 
	 * @param synsetID
	 * @return
	 */
	public static List<ISynsetID> getHyponyms(ISynsetID synsetID) {
		ISynset syn = dict.getSynset(synsetID);
		List<ISynsetID> hyponyms = syn.getRelatedSynsets(Pointer.HYPONYM);
		return makeSerializable(hyponyms);
	}

	/**
	 * Returns a list of lists of synset ids, each inner list specifying a path
	 * in the WordNet ontology from the given synset ID to the root concept.
	 * 
	 * @param synsetID
	 * @return
	 */
	public static List<List<ISynsetID>> hypernymsPaths(ISynsetID synsetID) {

		List<ISynsetID> hypernyms = getHypernyms(synsetID);

		List<List<ISynsetID>> newPaths = new ArrayList<List<ISynsetID>>();

		if (hypernyms.size() == 0) {
			List<ISynsetID> newPath = new ArrayList<ISynsetID>();
			newPath.add(synsetID);
			newPaths.add(newPath);
		}

		for (ISynsetID synid : hypernyms) {
			List<List<ISynsetID>> paths = hypernymsPaths(synid);

			for (int i = 0; i < paths.size(); i++) {
				List<ISynsetID> curPath = paths.get(i);
				curPath.add(0, synsetID); // append current synset
				newPaths.add(curPath);
			}
		}

		return newPaths;
	}

	/**
	 * Returns the glossary entry for the given synset.
	 * 
	 * @param synset
	 * @return
	 */
	public static String getGlossForSynset(ISynsetID synset) {

		return getSynsetFromID(synset).getGloss();
	}

	/**
	 * Returns the glossary entries for all possible word senses of the given
	 * word and part of speech.
	 * 
	 * @param word
	 * @param pos
	 * @return
	 */
	public static List<String> getGlossesForWord(String word, POS pos) {
		List<String> glosses = new ArrayList<String>();
		List<ISynsetID> senses = getSynsetIDs(word, pos);

		for (ISynsetID sense : senses) {
			glosses.add(getGlossForSynset(sense));
		}

		return glosses;
	}

	/**
	 * Returns the shortest path from on synset to the root synset
	 * 
	 * @param synid
	 * @return
	 */
	public static List<ISynsetID> getShortesHypernymsPath(ISynsetID synid) {
		List<List<ISynsetID>> paths = hypernymsPaths(synid);
		List<ISynsetID> shortestPath = null;

		for (int i = 0; i < paths.size(); i++) {
			if (shortestPath == null
					|| paths.get(i).size() < shortestPath.size())
				shortestPath = paths.get(i);
		}

		return shortestPath;
	}

	/**
	 * Removes the shared path to the root synset of two hypernyms paths
	 * 
	 * @param path1
	 * @param path2
	 * @return
	 */
	public static ISynsetID[] removeCommonPath(ISynsetID[] path1,
			ISynsetID[] path2) {
		if (dict == null)
			init();

		for (int i = 0; i < path1.length; i++) {
			for (int j = 0; j < path2.length; j++) {
				if (path1[i].equals(path2[j])) {
					int newlength = i + j + 1;
					ISynsetID[] combined = new ISynsetID[newlength];
					System.arraycopy(path1, 0, combined, 0, i + 1);
					for (int k = 0; k < j; k++)
						combined[i + k + 1] = path2[j - k - 1];
					return combined;
				}
			}
		}
		return new ISynsetID[0];
	}

	/**
	 * Returns the WUP-similarity of two synsets.
	 * 
	 * @param syn1
	 * @param syn2
	 * @return
	 */
	public static double getWUPSimilarity(ISynsetID syn1, ISynsetID syn2) {

		if (syn1.equals(syn2))
			return 1.;

		double depth1 = getMaxDepth(hypernymsPaths(syn1));
		double depth2 = getMaxDepth(hypernymsPaths(syn2));
		ISynsetID lca = getMostSpecificCommonAncestor(syn1, syn2);

		double simil;
		if (lca == null) {
			simil = 0;
		} else {
			double depthLCA = getMaxDepth(hypernymsPaths(lca));

			simil = depthLCA / (.5 * (depth1 + depth2));
		}
		return simil;
	}

	/**
	 * Returns the most specific common ancestor of <code>synset1</code> and
	 * <code>synset2</code>
	 * 
	 * @param synset1
	 * @param synset2
	 * @return
	 */
	public static ISynsetID getMostSpecificCommonAncestor(ISynsetID synset1,
			ISynsetID synset2) {
		if (dict == null)
			init();

		List<List<ISynsetID>> paths1 = hypernymsPaths(synset1);
		List<List<ISynsetID>> paths2 = hypernymsPaths(synset2);
		ISynsetID lcaSynset = null;
		int maxLcaDepth = -1;
		// System.out.println(WordNet3.getSynsetFromID(synset1) + ", " +
		// WordNet3.getSynsetFromID(synset2));
		for (int i = 0; i < paths1.size(); i++) {
			for (int j = 0; j < paths2.size(); j++) {
				ISynsetID lca = getLeastCommonAncestor(paths1.get(i),
						paths2.get(j));

				if (lca == null)
					return null;

				List<List<ISynsetID>> lcaPaths = hypernymsPaths(lca);
				int lcaDepth = getMaxDepth(lcaPaths);
				if (maxLcaDepth < 0 || lcaDepth > maxLcaDepth) {
					maxLcaDepth = lcaDepth;
					lcaSynset = lca;
				}
			}
		}

		return lcaSynset;
	}

	public static int getMaxDepth(List<List<ISynsetID>> paths) {
		int minDepth = -1;

		for (int i = 0; i < paths.size(); i++) {
			int l = paths.get(i).size() - 1;
			if (minDepth < 0 || l > minDepth)
				minDepth = l;
		}
		return minDepth;
	}

	public static ISynsetID getLeastCommonAncestor(List<ISynsetID> path1,
			List<ISynsetID> path2) {

		for (int i = 0; i < path1.size(); i++)
			for (int j = 0; j < path2.size(); j++)
				if (path1.get(i).equals(path2.get(j)))
					return path1.get(i);

		return null;
	}

	/**
	 * Returns the exemplary word of this synset.
	 * 
	 * @param synsetID
	 * @return
	 */
	public static String[] getWordExamples(ISynsetID synsetID) {
		if (dict == null)
			init();

		ISynset synset = dict.getSynset(synsetID);

		if (synset == null)
			return null;

		List<IWord> wordlist = synset.getWords();
		String[] words = new String[wordlist.size()];
		for (int i = 0; i < words.length; i++)
			words[i] = wordlist.get(i).getLemma();

		return words;
	}

	/**
	 * Checks whether concept is a subtype of superConcept
	 * 
	 * @param concept
	 * @param superConcept
	 * @return
	 */
	public static boolean isATransitive(ISynsetID concept,
			ISynsetID superConcept) {

		// Check the superConcept for membership in the concept hypernyms paths
		List<List<ISynsetID>> paths = hypernymsPaths(concept);
		for (List<ISynsetID> path : paths) {
			if (path.contains(superConcept))
				return true;
		}
		return false;
	}

	/**
	 * Computes the similarity of two concepts.
	 * 
	 * @param synset1
	 * @param synset2
	 * @return
	 */
	public static double getSynsetSimilarity(ISynsetID synset1,
			ISynsetID synset2) {

		if (subtreeHeights == null)
			computeSubtreeHeights();

		if (subtreeHeights.get(synset1) == null
				|| subtreeHeights.get(synset2) == null)
			return -1;

		// get the root height
		double h_R = getRootHeight();

		// get height of most specific common ancestor
		double a_S = getSubtreeHeight(getMostSpecificCommonAncestor(synset1,
				synset2)) / h_R;

		// get heights of synset1 and synset2
		double a_A = getSubtreeHeight(synset1) / h_R;
		double a_B = getSubtreeHeight(synset2) / h_R;

		double similarity = (1 - a_S) / (1 - (.5 * (a_A + a_B)));
		return similarity;
	}

	/**
	 * Reads the irregular verb forms from the file verb.exc and stores them in
	 * a cache.
	 * 
	 */
	private static void cacheIrregularVerbs() {
		File verb_exc = new File(wnhome + File.separator + "dict"
				+ File.separator + "verb.exc");

		irregularVerbs = new HashMap<String, String>();

		try {
			BufferedReader reader = new BufferedReader(new FileReader(verb_exc));

			String line;
			while ((line = reader.readLine()) != null) {

				String[] keyValueSet = line.split("\\s");
				if (keyValueSet.length >= 2) {
					// replace all underscores by spaces
					String value = keyValueSet[keyValueSet.length - 1]
							.replaceAll("_", " ");

					for (int i = 0; i < keyValueSet.length - 1; i++) {
						String key = keyValueSet[i].replaceAll("_", " ");
						irregularVerbs.put(key, value);
					}
				} else
					System.err.println("[WordNet]: WARNING: Illegal mapping "
							+ "entry in irregular verbs file: " + line);
			}
		} catch (Exception e) {
			System.err.println("[WordNet]: WARNING: Could not read "
					+ "irregular verbs file " + verb_exc.getPath() + ":");
			System.err.println(e.getMessage());
		}
	}

	/**
	 * Reads the irregular noun forms in the file noun.exc and stores them in a
	 * cache.
	 * 
	 */
	private static void cacheIrregularNouns() {
		File noun_exc = new File(wnhome + File.separator + "dict"
				+ File.separator + "noun.exc");

		irregularNouns = new HashMap<String, String>();

		try {
			BufferedReader reader = new BufferedReader(new FileReader(noun_exc));

			String line;
			while ((line = reader.readLine()) != null) {
				String[] keyValueSet = line.split("\\s");

				if (keyValueSet.length >= 2) {
					// replace all underscores by spaces
					String value = keyValueSet[keyValueSet.length - 1]
							.replaceAll("_", " ");
					for (int i = 0; i < keyValueSet.length; i++) {
						String key = keyValueSet[i].replaceAll("_", " ");
						irregularNouns.put(key, value);
					}
				} else
					System.err.println("[WordNet]: WARNING: Illegal "
							+ "mapping entry in irregular nouns file: " + line);
			}
			reader.close();

		} catch (Exception e) {
			System.err.println("[WordNet]: WARNING: Could not read "
					+ "irregular nouns file " + noun_exc.getPath() + ":");
			System.err.println(e.getMessage());
		}
	}

	/**
	 * Reads the irregular adjective form and stores them in the cache
	 */
	private static void cacheIrregularAdjectives() {
		File adj_exc = new File(wnhome + File.separator + "dict"
				+ File.separator + "adj.exc");

		irregularAdj = new HashMap<String, String>();

		try {
			BufferedReader reader = new BufferedReader(new FileReader(adj_exc));

			String line;
			while ((line = reader.readLine()) != null) {
				String[] keyValueSet = line.split("\\s");

				if (keyValueSet.length >= 2) {
					// replace all underscores by spaces
					String value = keyValueSet[keyValueSet.length - 1]
							.replaceAll("_", " ");
					for (int i = 0; i < keyValueSet.length; i++) {
						String key = keyValueSet[i].replaceAll("_", " ");
						irregularAdj.put(key, value);
					}
				} else
					System.err.println("[WordNet]: WARNING: Illegal "
							+ "mapping entry in irregular adjectives file: "
							+ line);
			}
		} catch (Exception e) {
			System.err.println("[WordNet]: WARNING: Could not"
					+ " read irregular adjectives file " + adj_exc.getPath()
					+ ":");
			System.err.println(e.getMessage());
		}
	}

	/**
	 * Reads the irregular adjective form and stores them in the cache
	 */
	private static void cacheIrregularAdverbs() {
		File adv_exc = new File(wnhome + File.separator + "dict"
				+ File.separator + "adv.exc");

		irregularAdv = new HashMap<String, String>();

		try {
			BufferedReader reader = new BufferedReader(new FileReader(adv_exc));

			String line;
			while ((line = reader.readLine()) != null) {
				String[] keyValueSet = line.split("\\s");

				if (keyValueSet.length >= 2) {
					// replace all underscores by spaces
					String value = keyValueSet[keyValueSet.length - 1]
							.replaceAll("_", " ");
					for (int i = 0; i < keyValueSet.length; i++) {
						String key = keyValueSet[i].replaceAll("_", " ");
						irregularAdv.put(key, value);
					}
				} else
					System.err.println("[WordNet]: WARNING: Illegal "
							+ "mapping entry in irregular adverbs file: "
							+ line);
			}
		} catch (Exception e) {
			System.err
					.println("[WordNet]: WARNING: Could not"
							+ " read irregular adverbs file "
							+ adv_exc.getPath() + ":");
			System.err.println(e.getMessage());
		}
	}

	/**
	 * Computes the height of the subtree given by each concept and caches it in
	 * a static map.
	 */
	protected static void computeSubtreeHeights() {
		subtreeHeights = new HashMap<ISynsetID, Integer>();
		// starting at the root ("entity")
		computeSubtreeHeights(new MySynsetID(1740, POS.NOUN));
	}

	/**
	 * Returns the sense keys associated to the given Synset ID
	 * 
	 * @param synsetid
	 * @return
	 */
	public static List<ISenseKey> getSenseKeys(ISynsetID synsetid) {
		ISynset synset = dict.getSynset(synsetid);
		List<ISenseKey> keys = new ArrayList<ISenseKey>();
		List<IWord> words = synset.getWords();

		for (IWord w : words)
			keys.add(w.getSenseKey());

		return keys;
	}

	/**
	 * Recursively traverses the concept taxonomy and caches the heights of
	 * subtrees in a map.
	 * 
	 * @param superconcept
	 * @return
	 */
	protected static int computeSubtreeHeights(ISynsetID superconcept) {

		List<ISynsetID> hyponyms = getHyponyms(superconcept);
		if (hyponyms.isEmpty()) {
			subtreeHeights.put(superconcept, 0);
			return 0;
		}

		int maxHeight = -1;
		for (int i = 0; i < hyponyms.size(); i++) {
			int height = 1 + computeSubtreeHeights(hyponyms.get(i));
			if (maxHeight == -1 || maxHeight < height)
				maxHeight = height;
		}

		if (!subtreeHeights.containsKey(superconcept)
				|| subtreeHeights.get(superconcept) < maxHeight)
			subtreeHeights.put(superconcept, maxHeight);

		return maxHeight;
	}

	/**
	 * Returns the height of the WordNet noun taxonomy.
	 * 
	 * @return
	 */
	public static int getRootHeight() {
		if (subtreeHeights == null)
			computeSubtreeHeights();

		return subtreeHeights.get(new MySynsetID(1740, POS.NOUN)); // this is
																	// "entity"
	}

	/**
	 * Returns the height of the subtree of the concept given by synset.
	 * 
	 * @param synset
	 * @return
	 */
	public static int getSubtreeHeight(ISynsetID synset) {
		if (subtreeHeights == null)
			computeSubtreeHeights();

		return subtreeHeights.get(synset);
	}

	/**
	 * To run this piece of code, make sure that the WNHOME variable is set to
	 * the path where you have installed WordNet
	 * 
	 * @param args
	 */
	public static void main(String[] args) {

		// System.out.println(getSynsetSimilarity(new SynsetID(3383948,
		// POS.NOUN), new SynsetID(4284002, POS.NOUN)));
		// System.out.println(getSynsetSimilarity(new SynsetID(7753592,
		// POS.NOUN),
		// new SynsetID(3001627, POS.NOUN)));
		System.out.println(WordNet3.getHypernyms(MySynsetID.parseSynsetID("SID-00001740-N")));
//		System.out.println(WordNet3.getSenseKeys("portion", POS.NOUN));
//		System.out.println(WordNet3.getSenseKeys("cut down", POS.VERB));
	}
}
