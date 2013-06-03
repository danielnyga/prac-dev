package edu.cs.tum.mturk.test;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.geom.Rectangle2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.category.DefaultCategoryDataset;
import org.json.JSONArray;
import org.json.JSONObject;

import com.lowagie.text.Document;
import com.lowagie.text.PageSize;
import com.lowagie.text.Rectangle;
import com.lowagie.text.pdf.DefaultFontMapper;
import com.lowagie.text.pdf.PdfContentByte;
import com.lowagie.text.pdf.PdfTemplate;
import com.lowagie.text.pdf.PdfWriter;

import edu.mit.jwi.item.ISynset;
import edu.mit.jwi.item.POS;
import edu.tum.cs.ias.wordnet.WordNet3;


/**
 * This class enables a graphical evaluation of the results
 * Diagrams can be generated, shown and exported
 * Latex Code can be generated
 * 
 * @author Stephan Epping
 *
 */

public class SampleStats extends JFrame{

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	public static final String[] __VERBS = {"ADD","CUT","FILL","FLIP","MIX","PLACE","POUR","PUT","SLICE","STIR"};
	public static final String __PATH = "data/live_2/";
	
	public SampleStats(String title, List<JFreeChart> charts)
	{
		super(title);
		
		JPanel panel = new JPanel(new GridLayout((int)Math.ceil(charts.size()/3.0), 2));
		
		for(JFreeChart chart : charts )
		{
			ChartPanel chartPanel = new ChartPanel(chart, false);
			chartPanel.setPreferredSize(new Dimension(512, 512));
			panel.add(chartPanel);
		}
		
		panel.setPreferredSize(new Dimension(1024,(charts.size()/2)*512));
    	setContentPane(panel);
    	
    	setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE); 
	}
	
	public static void main(String[] args) throws Exception 
	{

		List<JFreeChart> charts = new ArrayList<JFreeChart>();
		
		charts.add(createSensesOccurrency("then"));
		
		//charts.add(createMajorityUsability());
		//charts.add(createSingleUsability());
		//charts.add(createAssignmentsHistory());
		//charts.add(createSenseResultSample());
		
		//charts.add(createSensesOccurrency("cup"));
		//charts.add(createSensesOccurrency("stir"));
		//charts.add(createSensesOccurrency("add"));
		/*
		charts.add(createSensesOccurrency("keep"));
		charts.add(createSensesOccurrency("oats"));
		charts.add(createSensesOccurrency("vegetables"));
		charts.add(createSensesOccurrency("spatula"));
		charts.add(createSensesOccurrency("garlic"));
		charts.add(createSensesOccurrency("evenly"));
		charts.add(createSensesOccurrency("soft"));
		charts.add(createSensesOccurrency("let"));
		charts.add(createSensesOccurrency("water"));
		
		charts.add(createSensesOccurrency("pieces"));
		charts.add(createSensesOccurrency("turn"));
		charts.add(createSensesOccurrency("next"));
		charts.add(createSensesOccurrency("little"));
		charts.add(createSensesOccurrency("together"));
		charts.add(createSensesOccurrency("pour"));
		charts.add(createSensesOccurrency("bottom"));
		charts.add(createSensesOccurrency("gently"));
		charts.add(createSensesOccurrency("onion"));
		charts.add(createSensesOccurrency("time"));
		charts.add(createSensesOccurrency("slice"));
		
		charts.add(createSensesOccurrency("cup"));
		charts.add(createSensesOccurrency("well"));
		charts.add(createSensesOccurrency("top"));
		charts.add(createSensesOccurrency("get"));
		charts.add(createSensesOccurrency("now"));
		charts.add(createSensesOccurrency("side"));
		
		charts.add(createSensesOccurrency("way"));
		charts.add(createSensesOccurrency("heat"));
		charts.add(createSensesOccurrency("cook"));
		charts.add(createSensesOccurrency("stir"));
		charts.add(createSensesOccurrency("mix"));
		charts.add(createSensesOccurrency("cut"));
		charts.add(createSensesOccurrency("do"));
		
		/*
		for(JFreeChart chart : charts){
			chart.getCategoryPlot().getRangeAxis().setTickLabelFont(new Font("Arial", Font.BOLD, 14));
			chart.getCategoryPlot().getDomainAxis().setTickLabelFont(new Font("Arial", Font.BOLD, 14));
			
			exportPDF(chart, 1024, 512, new File(__PATH+"_charts/"+chart.getTitle().getText().replaceAll(" ", "_")+".pdf"));
		
		//show chart
		JFrame frame = new SampleStats("Charts",charts);
		frame.setVisible(true);
        frame.pack();
        RefineryUtilities.centerFrameOnScreen(frame);
        frame.setVisible(true);
        */
        System.out.println("--------------------------");
        //listDifferentSenses();
	}
	
	public static void listDifferentSenses() throws Exception
	{
		HashMap<String,HashMap<String,Integer>> senses = new HashMap<String,HashMap<String,Integer>>();
		
		for(String verb : __VERBS)
		{
			
			File directory = new File(__PATH+verb);
			File result = new File(directory.getAbsolutePath()+"/HITS.results");

			BufferedReader br = new BufferedReader(new FileReader(result));
			
			JSONArray results = new JSONArray(br.readLine());
			
			for(int i = 0;i < results.length();i++)
			{
				JSONObject an = results.getJSONObject(i);
				
				String word = an.getString("word").toLowerCase();
				if(!senses.containsKey(word))
					senses.put(word,new HashMap<String, Integer>());
				
				HashMap<String,Integer> wsenses = senses.get(word);
				
				String answer = getMaxAnswer(an.getJSONArray("results"));
				if(answer == null)
					answer = "Ambiguous";
						
				if(!wsenses.containsKey(answer))
					wsenses.put(answer, new Integer(0));
						
					Integer count = wsenses.get(answer);
					count++;
					wsenses.put(answer, count);
			}
			
			br.close();
		}
			
		for(String word : senses.keySet())
		{
			HashMap<String,Integer> wsenses = senses.get(word);
			
			if(wsenses.keySet().size() > 1)
			{
				System.out.println(word);
				//i have more senses
				for(String sense : wsenses.keySet())
				{
					System.out.println("     "+sense+": "+wsenses.get(sense));
				}
			}
		}
		
	}

	public static JFreeChart createMajorityUsability() throws Exception
	{
		int valid = 0;
		int nomatch = 0;
		int othertype = 0;
		int ambigious = 0;
		
		for(String verb : __VERBS)
		{
			HashMap<String,Integer> senses = new HashMap<String,Integer>();
			int vnomatch = 0;
			int vothertype = 0;
			int vvalid = 0;
			int vambigious = 0;
			
			File directory = new File(__PATH+verb);
			
			File result = new File(directory.getAbsolutePath()+"/HITS.results");

			BufferedReader br = new BufferedReader(new FileReader(result));
			
			JSONArray results = new JSONArray(br.readLine());
			
			for(int i = 0;i < results.length();i++)
			{
				JSONObject an = results.getJSONObject(i);
				
				String answer = getMaxAnswer(an.getJSONArray("results"));
				if(answer == null)
					vambigious++;
				else if(answer.startsWith("NOMATCH"))
					vnomatch++;
				else if(answer.startsWith("OTHERTYPE"))
				{
					vothertype++;
				}else{
					vvalid++;
					
					if(an.getString("word").equalsIgnoreCase(verb))
					{
						if(!senses.containsKey(answer))
							senses.put(answer,new Integer(1));
						else
						{
							Integer a = senses.get(answer);
							a++;
							senses.put(answer, a);
						}
					}
				}
			}
			
			System.out.println(">> Read "+verb+" "+results.length());
			System.out.println("Senses: "+senses);
			System.out.println("NoMatch "+nomatch+"  OtherType "+othertype+" Ambigious "+vambigious);
			
			nomatch += vnomatch;
			othertype += vothertype;
			valid += vvalid;
			ambigious += vambigious;
			
			br.close();
		}
		
		DefaultCategoryDataset dataset = new DefaultCategoryDataset();
		dataset.addValue(valid, "set1", "Valid");
		dataset.addValue(ambigious, "set1", "Ambiguous");
		dataset.addValue(nomatch, "set1", "No match");
		dataset.addValue(othertype, "set1", "Wrong POS");
		
		JFreeChart chart = ChartFactory.createBarChart("Usable Majority Results", "result", "annotations", dataset, PlotOrientation.VERTICAL, false, true, true);
    	return chart;
	}
	
	public static JFreeChart createSingleUsability() throws Exception
	{	
		int svalid = 0;
		int snomatch = 0;
		int sothertype = 0;
		
		for(String verb : __VERBS)
		{	
			File directory = new File(__PATH+verb);
			
			File result = new File(directory.getAbsolutePath()+"/HITS.results");

			BufferedReader br = new BufferedReader(new FileReader(result));
			
			JSONArray results = new JSONArray(br.readLine());
			
			for(int i = 0;i < results.length();i++)
			{
				JSONObject an = results.getJSONObject(i);
				
				for(int j = 0; j <  an.getJSONArray("results").length();j++)
				{
					JSONObject single = an.getJSONArray("results").getJSONObject(j);
					
					String sense = single.getString("sense");
					
					if(sense.startsWith("NOMATCH"))
						snomatch += single.getInt("votes");
					else if(sense.startsWith("OTHERTYPE"))
						sothertype += single.getInt("votes");
					else
						svalid += single.getInt("votes");
				}
			}
			
			br.close();
		}
		
		DefaultCategoryDataset dataset = new DefaultCategoryDataset();
		dataset.addValue(svalid, "set1", "Valid");
		dataset.addValue(snomatch, "set1", "No match");
		dataset.addValue(sothertype, "set1", "Wrong POS");

		JFreeChart chart = ChartFactory.createBarChart("Usable Single Annotations", "result", "annotations", dataset, PlotOrientation.VERTICAL, false, true, true);
    	return chart;
	}
	
	private static JFreeChart createSensesOccurrency(String word) throws Exception
	{
		word = word.toLowerCase();
		
		HashMap<String,Integer> senses = new HashMap<String,Integer>();
		
		for(String verb : __VERBS)
		{
			
			File directory = new File(__PATH+verb);
			File result = new File(directory.getAbsolutePath()+"/HITS.results");

			BufferedReader br = new BufferedReader(new FileReader(result));
			
			JSONArray results = new JSONArray(br.readLine());
			
			for(int i = 0;i < results.length();i++)
			{
				JSONObject an = results.getJSONObject(i);
				
					if(an.getString("word").toLowerCase().equalsIgnoreCase(word))
					{
						String answer = getMaxAnswer(an.getJSONArray("results"));
						
						/*
						if(word.equals("onion")){
							System.out.println("Onion "+an.getString("sentence")+" "+answer);
						}*/
						
						if(answer == null)
							answer = "Ambiguous";
						
						if(!senses.containsKey(answer))
							senses.put(answer, new Integer(0));
						
						Integer count = senses.get(answer);
						count++;
						senses.put(answer, count);
					}
			}
			
			br.close();
		}
		
		
		List<ISynset> syns = WordNet3.getSynsets(word, POS.NOUN);
		syns.addAll(WordNet3.getSynsets(word, POS.VERB));
		syns.addAll(WordNet3.getSynsets(word, POS.ADJECTIVE));
		syns.addAll(WordNet3.getSynsets(word, POS.ADVERB));
		
		
		DefaultCategoryDataset dataset = new DefaultCategoryDataset();
			
		int i = 0;
		for(String sense : senses.keySet())
		{
			for(ISynset s : syns)
			{
				if(s.getID().toString().equals(sense))
				{
					if(i==0)
					{
						System.out.println("\\hline \\textbf{"+word+"} & "+s.getID()+" & "+s.getGloss()+"\\\\");
					}else{
						System.out.println("\\cline{2-3}  & "+s.getID()+" & "+s.getGloss()+"\\\\");
					}
				}
			}
			i++;
		}
		System.out.println("\\cline{2-3}  & \\multicolumn{2}{l|}{...} \\\\"); 
		System.out.println("\\hline");
		for(String sense : senses.keySet())
		{
			//System.out.println(senses.get(sense) +" "+ word + " " + sense);
			dataset.addValue(senses.get(sense), word, sense);
		}
		//printVerticalTable(senses,word);
		//printHorizontalTable(senses);
		JFreeChart chart = ChartFactory.createBarChart("Sense Occurrences "+word, "senses", "annotations", dataset, PlotOrientation.VERTICAL, false, true, true);
		
		return chart;
	}
	
	public static void printVerticalTable(HashMap<String, Integer> senses,String word) {
		
		System.out.println("\\subtable[Sense Occurencies "+word+"]{");
		ArrayList<String>list = new ArrayList<String>(senses.keySet());
		System.out.println("\\begin{tabular}[h!]{|p{3cm}|p{3cm}|}");
		System.out.println("\\hline");
		System.out.println("\\textbf{Sense} & \\textbf{Occurrences}\\\\");
		Collections.sort(list);
		
		for(String sense : list)
		{
			System.out.println("\\hline");
			System.out.println(sense+" & "+senses.get(sense)+"\\\\");
		}
		System.out.println("\\hline");
		System.out.println("\\end{tabular}");
		System.out.println("}");
	}

	public static void printHorizontalTable(HashMap<String, Integer> senses) {
		String str = "|";
		for(int i=0;i<senses.keySet().size();i++)str += "l|";
		System.out.println("\\begin{tabular}[h!]{"+str+"}");
		String header = "";
		String occ = "";
		for(String sense : senses.keySet())
		{
			if(header == "")
				header += sense;
			else
				header += " & "+sense;
			
			if(occ == "")
				occ += senses.get(sense);
			else
				occ += " & "+senses.get(sense);
		}
		System.out.println("\\hline");
		System.out.println(header+"\\\\");
		System.out.println("\\hline");
		System.out.println(occ+"\\\\");
		System.out.println("\\hline");
		System.out.println("\\end{tabular}");
	}
	
	public static JFreeChart createSenseResultSample()
	{
		DefaultCategoryDataset dataset = new DefaultCategoryDataset();
		
		//SID-01027174-V SID-00640828-V SID-00182406-V
		dataset.addValue(3, "votes", "SID-00182406-V");
		
		dataset.addValue(1, "votes", "SID-00640828-V");
		
		dataset.addValue(1, "votes", "SID-01027174-V");
		
		
		System.out.println(new Font("Arial", Font.BOLD, 20));
		JFreeChart chart = ChartFactory.createBarChart3D("Summarized Result", null, null, dataset, PlotOrientation.VERTICAL, false, false, false);
		
		return chart;
	}
	
	public static JFreeChart createAssignmentsHistory()
	{
		DefaultCategoryDataset dataset = new DefaultCategoryDataset();
		
		dataset.addValue(0, "assigned", "1");
		dataset.addValue(23, "posted", "1");
		
		dataset.addValue(0, "assigned", "2");
		dataset.addValue(90, "posted", "2");
		
		dataset.addValue(2, "assigned", "3");
		dataset.addValue(23, "posted", "3");
		
		dataset.addValue(2, "assigned", "4");
		dataset.addValue(0, "posted", "4");
		
		dataset.addValue(14, "assigned", "5");
		dataset.addValue(0, "posted", "5");
		
		dataset.addValue(6, "assigned", "6");
		dataset.addValue(0, "posted", "6");
		
		dataset.addValue(3, "assigned", "7");
		dataset.addValue(0, "posted", "7");
		
		dataset.addValue(0, "assigned", "8");
		dataset.addValue(9, "posted", "8");
		
		dataset.addValue(10, "assigned", "9");
		dataset.addValue(0, "posted", "9");
		
		dataset.addValue(3, "assigned", "10");
		dataset.addValue(0, "posted", "10");
		
		dataset.addValue(18, "assigned", "11");
		dataset.addValue(0, "posted", "11");
		
		dataset.addValue(0, "assigned", "12");
		dataset.addValue(0, "posted", "12");
		
		dataset.addValue(1, "assigned", "13");
		dataset.addValue(0, "posted", "13");
		
		dataset.addValue(229, "assigned", "14");
		dataset.addValue(1416, "posted", "14");
		
		dataset.addValue(353, "assigned", "15");
		dataset.addValue(0, "posted", "15");
		
		dataset.addValue(283, "assigned", "16");
		dataset.addValue(0, "posted", "16");
		
		dataset.addValue(383, "assigned", "17");
		dataset.addValue(0, "posted", "17");
		
		dataset.addValue(1453, "assigned", "18");
		dataset.addValue(0, "posted", "18");
		
		dataset.addValue(335, "assigned", "19");
		dataset.addValue(0, "posted", "19");
		
		dataset.addValue(531, "assigned", "20");
		dataset.addValue(0, "posted", "20");
		
		dataset.addValue(1172, "assigned", "21");
		dataset.addValue(0, "posted", "21");
		
		dataset.addValue(1140, "assigned", "22");
		dataset.addValue(0, "posted", "22");
		
		dataset.addValue(1224, "assigned", "23");
		dataset.addValue(0, "posted", "23");
		
		JFreeChart chart = ChartFactory.createLineChart("History: Assignments a day", "days", "assignments", dataset, PlotOrientation.VERTICAL, true, true, true);
		
		chart.getCategoryPlot().setBackgroundPaint(Color.white); 
		chart.getCategoryPlot().getRenderer().setSeriesStroke(0, 
				new BasicStroke(
                5.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_MITER, 
                1.0f, null, 0.0f
            ));
		chart.getCategoryPlot().getRenderer().setSeriesStroke(1, 
				new BasicStroke(
                5.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_MITER, 
                1.0f, null, 0.0f
            ));
		return chart;
	}
	
	private static String getMaxAnswer(JSONArray r)
	{
		String maxAnswer = null;
		int maxAnswers = -1;
		int givenMaxes = 0;
		
		for(int i = 0; i < r.length();i++)
		{
			int v = r.getJSONObject(i).getInt("votes");
			
			if(v > maxAnswers)
			{
				maxAnswer = r.getJSONObject(i).getString("sense");
				maxAnswers = v;
				givenMaxes = 1;
			}else if (v == maxAnswers)
				givenMaxes ++;
		}
		
		if(givenMaxes > 1)
			return null;
		else
			return maxAnswer;
	}
	
	public static void exportPDF(JFreeChart chart,
			   int width, int height, File file) {
				
			   Document document = new Document(new Rectangle(width, height));
			   try 
			   {
			      PdfWriter writer;
			      writer = PdfWriter.getInstance(document, new FileOutputStream(file));
			      document.open();
			      document.addProducer();
			      document.addCreator("Stephan Epping");
			      document.addTitle(chart.getTitle().getText());
			      document.setPageSize(PageSize._11X17);

			      PdfContentByte cb = writer.getDirectContent();
			      PdfTemplate tp = cb.createTemplate(width, height);
			      Graphics2D g2d = tp.createGraphics(width, height, new DefaultFontMapper());
			      Rectangle2D r2d = new Rectangle2D.Double(0, 0, width, height);
			      chart.draw(g2d, r2d);
			      g2d.dispose();
			      cb.addTemplate(tp, 0, 0);
			   }
			   catch(Exception e) {
			      e.printStackTrace();
			   }
			   document.close();
			}


}
