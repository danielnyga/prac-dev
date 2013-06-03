package edu.cs.tum.mturk.test;

import java.io.File;
import java.io.IOException;
import java.text.ParseException;

import edu.cs.tum.mturk.MTurkWorkerPool;
import edu.cs.tum.mturk.model.MTurkWorker;

/**
 * This file merges two workerpools together
 * 
 * @author Stephan Epping
 *
 */

public class MergePools {

	/**
	 * @param args
	 * @throws ParseException 
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException, ParseException {
		File f = new File("data/live_2/_worker/");
		
		MTurkWorkerPool pool = new MTurkWorkerPool("data/live_2/WORKER.pool");
		pool.open();
		
		for(File pf : f.listFiles())
		{
			if(pf.getName().endsWith(".pool"))
			{
				MTurkWorkerPool p = new MTurkWorkerPool(pf.getAbsolutePath());
				p.open();
				
				for(String workerId : p.getWorkerIds())
				{
					MTurkWorker w = p.getWorker(workerId);
					MTurkWorker worker = pool.getWorker(workerId);
					
					worker.setRightAnswers(worker.getRightAnswers()+w.getRightAnswers());
					worker.setFalseAnswers(worker.getFalseAnswers()+w.getFalseAnswers());
					worker.setBlocked(worker.isBlocked()||w.isBlocked());
				}
				
				p.close();
			}
		}
		
		pool.close();

	}

}
