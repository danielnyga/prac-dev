package edu.cs.tum.mturk.model;

import java.io.IOException;
import java.util.Properties;

import com.amazonaws.mturk.addon.HITProperties;

/**
 * This class extends the standard HITProperties by providing
 * furhter properties for an WSD task
 * 
 * @author Stephan Epping
 *
 */

public class MTurkProperties extends HITProperties {

	private String limit;
	private String wrongPOS;
	private String nomatch;
	
	public MTurkProperties(Properties props) {
		super(props);
	}
	
	public MTurkProperties(String props) throws IOException{
		super(props);
	}
	
	/**
	 * Returns the default WSD task properties
	 * 
	 * @return properties
	 */
	public static MTurkProperties getDefaults(){
		
		MTurkProperties properties = new MTurkProperties(new Properties());

		properties.setTitle("Word Sense Disamibuation Task");
		properties.setKeywords("annotate,tag,sense,word,wordsense,ias");
		properties.setDescription("Annotate a word sense in a specific textual context");
		properties.setAnnotation("This HIT created by IAS Munich");
		
		properties.setRewardAmount("0.01");
		
		properties.setMaxAssignments("1");
		
		properties.setAssignmentDuration("120");
		properties.setLifetime("259200");//3 days in seconds
		properties.setAutoApprovalDelay("2592000");//30 days in seconds
		
		//Approval Rate > 95%
		properties.setQualificationType(1, "000000000000000000L0");
		properties.setQualificationComparator(1, "greaterthan");
		properties.setQualificationValue(1, "95");
		properties.setQualificationPrivate(1, "false");
		
		//00000000000000000040 = number of HIT of the worker to have been approved
		
		return properties;
	}

	/**
	 * Limit of possible answers
	 * 
	 * @return number of answers
	 */
	public int getLimit() 
	{
		if (limit == null)
		    return -1;

		return Integer.parseInt(limit);
	}

	public void setLimit(String limit) {
		this.limit = limit;
	}

	/**
	 * Provides the additional answer if not the right POS was tagged
	 * 
	 * @return true, if task should offer additional answer
	 */
	public boolean isOthertype() {
		if (wrongPOS == null)
		    return false;

		return Boolean.parseBoolean(wrongPOS);
	}

	public void setOthertype(String othertype) {
		this.wrongPOS = othertype;
	}

	/**
	 * Provides the additional answer if no sense matches
	 * @return true, if the additional answer should be offered
	 */
	public boolean isNomatch() {
		if (nomatch == null)
		    return false;

		return Boolean.parseBoolean(nomatch);
	}

	public void setNomatch(String nomatch) {
		this.nomatch = nomatch;
	}
	
	@Override
	protected void populateFields(Properties props) {
		super.populateFields(props);
		
		//Read properties from source
		setLimit(props.getProperty("answerlimit"));
		setOthertype(props.getProperty("showwrongposoption"));
		setNomatch(props.getProperty("shownomatchoption"));
	
		System.out.println("Properties "+getTitle()+":limit="+getLimit()+" showothertype="+isOthertype()+" shownomatch="+isNomatch());
	}

}
