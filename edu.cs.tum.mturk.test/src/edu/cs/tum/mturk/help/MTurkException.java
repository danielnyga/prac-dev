package edu.cs.tum.mturk.help;

public class MTurkException extends Exception {

	/**
	 * Ordinary Exception identifes problem with MTurk
	 */
	private static final long serialVersionUID = -8276358911318960197L;

	public MTurkException() {
		super();
	}

	public MTurkException(String message, Throwable cause) {
		super(message, cause);
	}

	public MTurkException(String message) {
		super(message);
	}

	public MTurkException(Throwable cause) {
		super(cause);
	}

}
