package edu.cs.tum.mturk;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;
import java.util.HashMap;
import java.util.Set;

import edu.cs.tum.mturk.help.IMTurkParser;
import edu.cs.tum.mturk.help.MTurkParser;
import edu.cs.tum.mturk.model.MTurkWorker;

/**
 * This class maintains some information about traced worker
 * 
 * @author Stephan Epping
 *
 */
public class MTurkWorkerPool {
	
	private HashMap<String,MTurkWorker> pool;
	private String destination = "defaultworker.pool";
	private IMTurkParser parser = null;
	
	/**
	 * Create an empty workerpool
	 */
	public MTurkWorkerPool(String destination){
		super();
		this.destination = destination;
		pool = new HashMap<String, MTurkWorker>();
		parser = new MTurkParser();
	}
	
	/**
	 * Open a new or existing workerpool
	 * 
	 * @throws IOException
	 * @throws ParseException
	 */
	public void open() throws IOException, ParseException 
	{	
		File poolFile = new File(this.destination);
	
		//Check if file exists
		if(poolFile.exists())
		{	
			//Read existing pool data
			BufferedReader bf = new BufferedReader(new FileReader(poolFile));
	
			String line = null;
			StringBuffer result = new StringBuffer();
		
			while((line = bf.readLine())!=null)
				result.append(line);
	
			bf.close();
		
			//Parse pool data to pool object
			pool = parser.parseWorkerPool(result.toString());
		}
	}
	
	/**
	 * Saves the workerpool to DISK
	 * Do not forget to close the pool!
	 * 
	 * @throws IOException
	 */
	public void close() throws IOException
	{
		File poolFile = new File(this.getDestination());
		
		BufferedWriter writer = new BufferedWriter(new FileWriter(poolFile));
		
		//Parse object to saveable format
		String json = parser.parseWorkerPool(this);
		
		writer.write(json);
		writer.flush();
		writer.close();
	}
	
	/**
	 * List of workerIds
	 * 
	 * @return a list of workerIds in this pool
	 */
	public Set<String> getWorkerIds(){
		return pool.keySet();
	}
	
	/**
	 * Returns a workerobject
	 * (If workerId not in pool an new one will be created)
	 * 
	 * @param workerId identifies the worker
	 * 
	 * @return worker object
	 */
	public MTurkWorker getWorker(String workerId)
	{	
		if(pool.containsKey(workerId))
			return pool.get(workerId);
		
		//Create new worker
		MTurkWorker worker = new MTurkWorker(workerId);
		pool.put(workerId, worker);
		
		return worker;
	}

	//General getter/setter
	
	public HashMap<String, MTurkWorker> getPool() {
		return pool;
	}

	public void setPool(HashMap<String, MTurkWorker> pool) {
		this.pool = pool;
	}

	public String getDestination() {
		return destination;
	}

	public void setDestination(String destination) {
		this.destination = destination;
	}
}
