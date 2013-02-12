package edu.tum.cs.ias.framenet;

import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.jdom.Attribute;
import org.jdom.Document;
import org.jdom.Element;
import org.jdom.JDOMException;
import org.jdom.Namespace;
import org.jdom.input.SAXBuilder;
import org.jdom.xpath.XPath;

public class FrameNet {

	/** The xml file name of the fulltext annotations */
	public static final String FULLTEXT_INDEX_XML = "fulltextIndex.xml";

	/** the directory name where the fulltext xmls are located */
	public static final String FULLTEXT_DIR = "fulltext";

	/** the xml file name of the frame index file */
	public static final String FRAME_INDEX_XML = "frameIndex.xml";

	protected Document fulltextIndex = null;
	protected Document frameIndex = null;

	/** Path to the FrameNet data location */
	protected String fnData = null;

	protected SAXBuilder builder = null;

	/**
	 * Creates a new FrameNet object. <code>fnData</code> is path pointing to
	 * the directory where the local FrameNet data is located.
	 * 
	 * @param fnData
	 */
	public FrameNet(String fnData) {
		try {
			this.fnData = fnData;
			builder = new SAXBuilder();
			fulltextIndex = builder.build(new File(fnData + File.separatorChar
					+ FULLTEXT_INDEX_XML));

			frameIndex = getFramesDocument();

			if (frameIndex == null)
				System.err.println("WARNING: Could not load FrameNet data!");

		} catch (JDOMException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Returns a list of all frames contained in FrameNet
	 * 
	 * @return
	 */
	@SuppressWarnings("unchecked")
	public List<String> getFrames() {
		List<String> frameList = new ArrayList<String>();

		try {
			Namespace ns = frameIndex.getRootElement().getNamespace();
			XPath xpath;
			xpath = XPath.newInstance("//ns:frameIndex/ns:frame/@name");
			xpath.addNamespace("ns", ns.getURI());

			List<Attribute> names = xpath.selectNodes(frameIndex);

			for (Attribute e : names) {
				//System.out.println(e.getValue());
				frameList.add(e.getValue());
			}
		} catch (JDOMException e1) {
			e1.printStackTrace();
		}

		return frameList;
	}

	/**
	 * Returns a list of all lexical units of a frame.
	 * 
	 * @param frame
	 * @return
	 */
	@SuppressWarnings("unchecked")
	public List<String> getLexicalUnits(String frame) {
		List<String> frameElements = new ArrayList<String>();

		try {
			Namespace ns = frameIndex.getRootElement().getNamespace();
			XPath xpath = XPath.newInstance("//ns:frameIndex/ns:frame[@name='"
					+ frame + "']/ns:lexUnit/@name");
			xpath.addNamespace("ns", ns.getURI());

			List<Attribute> feNames = xpath.selectNodes(frameIndex);

			for (Attribute e : feNames) {
			//	System.out.println(e.getValue());
				frameElements.add(e.getValue());
			}
		} catch (JDOMException e1) {
			e1.printStackTrace();
		}

		return frameElements;
	}

	/**
	 * Returns a list of all frame elements of the given frame.
	 * 
	 * @param frame
	 * @return
	 */
	public List<String> getFrameElements(String frame) {
		List<String> frameElements = new ArrayList<String>();

		try {
			Namespace ns = frameIndex.getRootElement().getNamespace();
			XPath xpath = XPath.newInstance("//ns:frameIndex/ns:frame[@name='"
					+ frame + "']/ns:FE/@name");
			xpath.addNamespace("ns", ns.getURI());

			@SuppressWarnings("unchecked")
			List<Attribute> feNames = xpath.selectNodes(frameIndex);

			for (Attribute e : feNames) {
			//	System.out.println(e.getValue());
				frameElements.add(e.getValue());
			}
		} catch (JDOMException e1) {
			e1.printStackTrace();
		}

		return frameElements;
	}

	/**
	 * This method returns an index of all frames. It combines the
	 * framesIndex.xml in the FrameNet data with the frame/* data.
	 * 
	 * @return
	 */
	protected Document getFramesDocument() {
		try {
			Document frameIndexTmp = builder.build(new File(fnData
					+ File.separatorChar + FRAME_INDEX_XML));

			// We remove all children of the frame index...
			Element root = frameIndexTmp.getRootElement();
			root.removeContent();

			File frameDir = new File(fnData + File.separatorChar + "frame");
			File[] frameFiles = frameDir.listFiles(new FilenameFilter() {

				@Override
				public boolean accept(File dir, String name) {
					if (name.endsWith(".xml"))
						return true;
					return false;
				}
			});

			for (File f : frameFiles) {
				List<Element> frames = getFrame(f.getPath());
				root.addContent(frames);
			}

			return frameIndexTmp;
		} catch (JDOMException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return null;
	}

	/**
	 * Returns the "frame" elements of a given xml file.
	 * 
	 * @param strFileName
	 * @return
	 */
	@SuppressWarnings("unchecked")
	protected List<Element> getFrame(String strFileName) {
		try {
			Document frameDocument = builder.build(new File(strFileName));
			Namespace ns = frameDocument.getRootElement().getNamespace();

			XPath xpath = XPath.newInstance("//ns:frame");
			xpath.addNamespace("ns", ns.getURI());

			List<Element> frames = xpath.selectNodes(frameDocument);

			// Remove all frame from the parent document such that we can add to
			// the frameIndex document
			for (Element frame : frames)
				frame.getDocument().removeContent(frame);

			return frames;
		} catch (JDOMException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return new ArrayList<Element>();
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		System.out.println("Testing FrameNet Query...");
		FrameNet fn = new FrameNet("/work/fndata-1.5");
		
		List<String> frames = fn.getFrames();
		List<String> frameElements = fn.getFrameElements("Cutting");
		List<String> lexicalUnits = fn.getLexicalUnits("Cutting");
		//System.out.println(frames);
	}

}
