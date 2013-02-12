package edu.cs.tum.mturk.model;

/**
 * This class represents an ordinary worker and provides a few statsitics for each one
 * 
 * @author Stephan Epping
 *
 */
public class MTurkWorker {

	private String workerId;
	
	private int falseAnswers;
	private int rightAnswers;
	
	private boolean blocked;
	
	public MTurkWorker(String workerId){
		super();
		this.workerId = workerId;
		blocked = false;
	}
	
	public boolean isBlocked() {
		return blocked;
	}

	public void setBlocked(boolean blocked) {
		this.blocked = blocked;
	}

	public void incrementRight(){
		rightAnswers++;
	}
	
	public void incrementFalse(){
		falseAnswers++;
	}
	
	public String getWorkerId() {
		return workerId;
	}
	public void setWorkerId(String workerId) {
		this.workerId = workerId;
	}
	public int getFalseAnswers() {
		return falseAnswers;
	}
	public void setFalseAnswers(int falseAnswers) {
		this.falseAnswers = falseAnswers;
	}
	public int getRightAnswers() {
		return rightAnswers;
	}
	public void setRightAnswers(int rightAnswers) {
		this.rightAnswers = rightAnswers;
	}
}
