package edu.cs.tum.mturk.model;

public class MTurkWord {

	private String value;
	private String tag;
	private int index;
	
	public MTurkWord(String value, String tag, int index) {
		super();
		this.value = value;
		this.tag = tag;
		this.index = index;
	}

	public String getValue() {
		return value;
	}

	public void setValue(String value) {
		this.value = value;
	}

	public String getTag() {
		return tag;
	}

	public void setTag(String tag) {
		this.tag = tag;
	}

	public int getIndex() {
		return index;
	}

	public void setIndex(int index) {
		this.index = index;
	}
	
}
