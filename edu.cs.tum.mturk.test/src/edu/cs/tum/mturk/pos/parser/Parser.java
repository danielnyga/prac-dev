package edu.cs.tum.mturk.pos.parser;

import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

import edu.mit.jwi.item.POS;
import edu.stanford.nlp.parser.lexparser.LexicalizedParser;
import edu.stanford.nlp.trees.GrammaticalStructure;
import edu.stanford.nlp.trees.GrammaticalStructureFactory;
import edu.stanford.nlp.trees.PennTreebankLanguagePack;
import edu.stanford.nlp.trees.Tree;
import edu.stanford.nlp.trees.TreebankLanguagePack;
import edu.stanford.nlp.trees.TypedDependency;
import edu.stanford.nlp.util.Filter;
import edu.stanford.nlp.util.ScoredObject;
import edu.tum.cs.ias.wordnet.WordNet3;

public class Parser {
	String fileToParse;

	String outputFile;

	LexicalizedParser lp;

	ArrayList<String> sentences = new ArrayList<String>();

	private int usePreProcessor = 0;

	public static final int PRE_PROCESSOR_NONE = 0;

	public static final int PRE_PROCESSOR_QUOTATION_MARKS = 1;

	public static final int PRE_PROCESSOR_LOWER_CASE = 2;

	public static final int PRE_PROCESSOR_USE_CIPHERS_ONLY = 4;

	public Parser (String location) {

		// "@LOCAL_PACKAGE_PATH@/etc/englishPCFG.ser.gz"
		lp = new LexicalizedParser(location, new String[] { "-maxLength",
				"160", "-retainTmpSubcategories" });
//		lp.setOptionFlags(new String[] { "-maxLength", "160",
//				"-retainTmpSubcategories" });
	}

	public void usePreProcessor( int preProcessor ) {

		usePreProcessor = preProcessor;
	}

	public String PreProcessorQuotationMarks( String sentence ) {

		return "\"" + sentence.replaceAll( "\\.", "" ) + "!\"";
	}

	public String PreProcessorLowerCase( String sentence ) {

		return sentence.toLowerCase();
	}

	public String PreProcessorUseCiphersOnly( String sentence ) {

		sentence = sentence.replaceAll( " one |One ", " 1 " );
		sentence = sentence.replaceAll( " two |Two ", " 2 " );
		sentence = sentence.replaceAll( " three |Three ", " 3 " );
		sentence = sentence.replaceAll( " four |Four", " 4 " );
		sentence = sentence.replaceAll( " five |Five ", " 5 " );
		sentence = sentence.replaceAll( " six |Six ", " 6 " );
		sentence = sentence.replaceAll( " seven |Seven ", " 7 " );
		sentence = sentence.replaceAll( " eight |Eight ", " 8 " );
		sentence = sentence.replaceAll( " nine |Nine ", " 9 " );
		sentence = sentence.replaceAll( " ten |Ten ", " 10 " );
		sentence = sentence.replaceAll( " eleven |Eleven ", " 11 " );
		sentence = sentence.replaceAll( " twelve |Twelve ", " 12 " );
		return sentence;
	}

	public Iterator<String> iterator() {

		File file = new File( fileToParse );

		try {
			// Read file to parse.
			String parseString = "";
			FileReader reader = new FileReader( file );
			int read = reader.read();

//			if ( outputFile != null ) {
//				File out = new File( outputFile );
//				if ( out.exists() )
//					out.delete();
//			}

			while ( read >= 0 ) {
				parseString += (char) read;
				read = reader.read();
			}
			reader.close();
			// System.out.println("================\nParsing String:\n" +
			// parseString);

			// Parse every sentence since Standford Parser
			// can only cope with single sentences.
			parseString = parseString.replaceAll( "\r", "" );
			String[] sent = parseString.split( "\n" );
			sentences.clear();

			for ( int i = 0; i < sent.length; i++ ) {
				sentences.add( sent[i] );
			}

			return sentences.iterator();
		}
		catch ( Exception e ) {
			e.printStackTrace();
		}
		
		return null;
	}

	// public SyntaxTree parseFile() {
	//			
	// SyntaxTree globalTree = new SyntaxTree();
	// globalTree.setElement( new SyntaxElement( "HOWTO", "" ) );
	//
	// for ( int i = 0; i < sentences.length; i++ ) {
	// System.out.println( "Parsing Sentence " + ( i + 1 ) + " of " +
	// sentences.length + ": "
	// + sentences[i] );
	// SyntaxTree tree = parse( sentences[i].trim() );
	//
	// globalTree.addChild( tree.getChildren().get( 0 ) );
	//
	// }
	// // If an output file is specified
	// // write the parsed SyntaxTree to it.
	// if ( outputFile != null ) {
	// File outFile = new File( outputFile );
	//
	// if ( ! outFile.exists() )
	// outFile.createNewFile();
	// FileWriter writer = new FileWriter( outFile );
	// writer.write( "" );
	//
	// globalTree.writeToFile( writer );
	// writer.close();
	// }
	//
	// return globalTree;
	//
	// return null;
	// }

	public String preprocess(String sentence) {
		String strToParse = sentence;

		if ( ( usePreProcessor & PRE_PROCESSOR_LOWER_CASE ) > 0 )
			strToParse = PreProcessorLowerCase( strToParse );

		if ( ( usePreProcessor & PRE_PROCESSOR_QUOTATION_MARKS ) > 0 )
			strToParse = PreProcessorQuotationMarks( strToParse );

		if ( ( usePreProcessor & PRE_PROCESSOR_USE_CIPHERS_ONLY ) > 0 )
			strToParse = PreProcessorUseCiphersOnly( strToParse );

		return strToParse;
	}
	
//	public List<SyntaxTree> kBestParses(String sentence, int k) {
//		
//		List<SyntaxTree> trees = new ArrayList<SyntaxTree>();
//		
//		String strToParse = preprocess(sentence);
//		
//		lp.apply(strToParse);
//		List<ScoredObject<Tree>> scoredObjects = lp.getKBestPCFGParses(k);
//		
//		for (ScoredObject<Tree> scoredObject: scoredObjects) {
//			Tree t = (Tree) scoredObject.object();
//			String pennString = t.pennString().replaceAll("\\[.+?\\]", "");
//			ArrayList<String> tokens = tokenize(pennString);
//			SyntaxTree syntaxTree = generateTree(null, tokens);
//			trees.add(syntaxTree);
//		}
//		
//		return trees;
//	}
	
	public SyntaxTree parse( String sentence ) {

		String returnString = "";

		String strToParse = preprocess(sentence);
		
		Tree parse = (Tree) lp.apply( strToParse );
		returnString = parse.pennString().replaceAll( "\\[.+?\\]", "" );
//		returnString = sentence;
//		System.out.println(returnString);

		
		ArrayList<String> tokens = tokenize( returnString );
		SyntaxTree tree = new SyntaxTree();

		tree = generateTree( null, tokens );

		return tree;
	}
	
//	public SyntaxTree parseWithConsistencyCheck(String sentence, int k) {
//		String strToParse = preprocess(sentence);
//		
//		List<SyntaxTree> trees = kBestParses(strToParse, k);
//		
//		SyntaxTree returnTree = null;
//		for (SyntaxTree tree: trees) {
//			if (checkConsistency(tree)) {
//				returnTree = tree;
//				break;
//			}
//		}
//		
//		if (returnTree == null)
//			returnTree = trees.get(0);
//		
//		return returnTree;
//	}
	
	/**
	 * Returns the set of typed depenedecies in the given sentence.
	 * @param sentence
	 * @return a collection of dependencies
	 */
	public Collection<TypedDependency> getDependencies(String sentence) {
		Tree parse = lp.apply(sentence);
		
		TreebankLanguagePack tlp = new PennTreebankLanguagePack();
		Filter<String> puncWordFilter = tlp.punctuationWordRejectFilter();
		GrammaticalStructureFactory gsf = tlp.grammaticalStructureFactory(puncWordFilter);
		GrammaticalStructure gs = gsf.newGrammaticalStructure(parse);
		Collection<TypedDependency> deps = gs.typedDependencies();
		
		return deps;
	}
	
	public Collection<TypedDependency> getCollapsedDependencies(String sentence) {
		Tree parse = lp.apply(sentence);
		
		TreebankLanguagePack tlp = new PennTreebankLanguagePack();
		Filter<String> puncWordFilter = tlp.punctuationWordRejectFilter();
		GrammaticalStructureFactory gsf = tlp.grammaticalStructureFactory(puncWordFilter);
		GrammaticalStructure gs = gsf.newGrammaticalStructure(parse);
		Collection<TypedDependency> deps = gs.typedDependenciesCollapsed();
		
		return deps;
	}
	
	public SyntaxTree generateFromTreeString(String tree) {
		
		ArrayList<String> tokens = tokenize( tree );
		return generateTree( null, tokens );
	}

	public SyntaxTree generateTree( SyntaxTree tree, ArrayList<String> tokens ) {

		SyntaxTree returnTree = null;

		for (Iterator<String> i = tokens.iterator(); i.hasNext(); ) {
			String token = (String) i.next();
			SyntaxTree node = new SyntaxTree();

			// Extract name and type of tree node
			String type = token.substring( 0, token.indexOf( " " ) ).trim();
			token = token.substring( token.indexOf( " " ) + 1, token.length() );
			String name = token.substring( 0,
					token.indexOf( "(" ) == - 1 ? token.length() : token.indexOf( "(" ) ).trim();

			// generate new sytax tree node
			SyntaxElement el = new SyntaxElement( type, name );
			node.setElement( el );
			node.setParent( tree );
			if ( tree != null )
				tree.addChild( node );
			returnTree = node;
			ArrayList<String> subTokens = tokenize( token );
			if ( subTokens.size() > 0 )
				generateTree( node, subTokens );
		}
		return returnTree;
	}

	public ArrayList<String> tokenize( String str ) {

		int stack = 0;
		int start = - 1;
		ArrayList<String> ret = new ArrayList<String>();

		for ( int i = 0; i < str.length(); i++ ) {
			if ( str.charAt( i ) == '(' ) {
				if ( stack == 0 )
					start = i + 1;
				stack++;
			}
			else if ( str.charAt( i ) == ')' ) {
				stack--;
				if ( stack == 0 ) {
					String token = str.substring( start, i );
					ret.add( token );
				}
			}
		}
		return ret;
	}
	
	public static boolean checkConsistency(SyntaxTree tree) {
		if (tree.getChildren().isEmpty()) 
			return checkConsistency(tree.getElement());
		boolean isConsistent = true;
		for (SyntaxTree child: tree.getChildren()) 
			isConsistent &= checkConsistency(child);
		
		return isConsistent;
	}

	/**
	 * Checks if a given leaf, i.e. a SyntaxElement, in a syntax tree has a consistent label 
	 * @param element
	 * @return true, is consistent
	 */
	public static boolean checkConsistency(SyntaxElement element) {
		POS pos = WordNet3.convertTagToWordnetPOS(element.type);
		if (pos == null)
			return true;
		boolean retVal = WordNet3.wordExistsAs(element.name, pos);
		if (!retVal)
			System.err.println(element.name + " " + element.type);
		return retVal;
	}
	
	public void setFileToParse( String fileToParse ) {

		this.fileToParse = fileToParse;
	}

	public String getFileToParse() {

		return fileToParse;
	}

	public String getOutputFile() {

		return outputFile;
	}

	public void setOutputFile( String outputFile ) {

		this.outputFile = outputFile;
	}
	
	public static void main(String[] args) {
		Parser p = new Parser("etc/englishPCFG.ser.gz");
		//SyntaxTree tree = p.parseWithConsistencyCheck("Pour blueberry mixture into prepared pie crust.", 10000);
		//System.out.println(tree);
//		LexicalizedParser p = new LexicalizedParser("./etc/englishPCFG.ser.gz");
		
		Collection<TypedDependency> deps = p.getDependencies("Place the cup on the table.");
		
		for (TypedDependency dep: deps) {
			System.out.println(dep.reln() + ": " + dep.dep() + " => " + dep.gov());
		}
	}
}
