/*******************************************************************************
 * Copyright (c) 2013 EclipseSource and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    Ralf Sternberg - initial API and implementation
 ******************************************************************************/
package prac.rap.main;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.jface.dialogs.InputDialog;
import org.eclipse.rap.addons.d3chart.ForceDirectedGraph;
import org.eclipse.rap.addons.d3chart.Link;
import org.eclipse.rap.addons.d3chart.Node;
import org.eclipse.rap.json.JsonObject;
import org.eclipse.rap.rwt.application.AbstractEntryPoint;
import org.eclipse.rap.rwt.service.ServerPushSession;
import org.eclipse.rap.rwt.widgets.DialogCallback;
import org.eclipse.rap.rwt.widgets.DialogUtil;
import org.eclipse.swt.SWT;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.SelectionListener;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.GridData;
import org.eclipse.swt.layout.GridLayout;
import org.eclipse.swt.layout.RowLayout;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.MessageBox;
import org.eclipse.swt.widgets.Slider;
import org.eclipse.swt.widgets.Text;
//import org.python.core.Py;
//import org.python.core.PyObject;
//import org.python.core.PyString;
//import org.python.core.PySystemState;
//import org.python.util.PythonInterpreter;





import prac.rap.data.MLNDB;
import prac.rap.data.MLNDB.Atom;
import ros.NodeHandle;
import ros.Ros;
import ros.RosException;
import ros.ServiceClient;
import ros.Subscriber;
import ros.pkg.rosmln.msg.AtomProbPair;
import ros.pkg.rosmln.msg.MLNDatabase;
import ros.pkg.rosprac.srv.PRACInfer;
import ros.pkg.rosprac.srv.PRACInfer.Response;

//import ros.pkg.rosmln.msg.MLNInference;

public class PRACMain extends AbstractEntryPoint {

	private int dbIndex = 0;
	
	public ForceDirectedGraph graph = null;

	public List<MLNDB> dbs = new ArrayList<MLNDB>();
	int dbCounter = 0;
	int nodeCounter = 0;
	public Map<String, Node> id2node = new HashMap<String, Node>();
	public Map<String[], Link> nodes2link = new HashMap<String[], Link>();
	public List<String> pracModules = new ArrayList<String>();
	
	private static final String PRAC_INFER = "PRACInfer";

	
	private static final int ROS_EXCEPTION = -1;

	public void initPracModules() {
		pracModules.add("nl_parsing");
		pracModules.add("ac_recognition");
		pracModules.add("senses_and_roles");
	}
	@Override
	protected void createContents(Composite parent) {
		parent.setLayout(new GridLayout(1, true));

		graph = new ForceDirectedGraph(parent, SWT.NONE);

		GridData graphData = new GridData(SWT.FILL, SWT.FILL, true, true);
		graph.setLayoutData(graphData);

		Composite controlPanel = new Composite(parent, SWT.NONE);

		GridData panelData = new GridData(SWT.FILL, SWT.FILL, false, false);
		controlPanel.setLayoutData(panelData);

		controlPanel.setLayout(new RowLayout(SWT.HORIZONTAL));
		Button btnStart = new Button(controlPanel, SWT.PUSH);
		btnStart.setText("Start");

		btnStart.addSelectionListener(new SelectionListener() {

			public void widgetSelected(SelectionEvent e) {
				showMessageBox("Start PRAC Reasoning", "A PRAC Reasoning graph will be created", SWT.OK);
			}

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		Button btnAddNode = new Button(controlPanel, SWT.PUSH);
		btnAddNode.setText("Add a Database");
		btnAddNode.addSelectionListener(new SelectionListener() {

			public void widgetSelected(SelectionEvent e) {
				graph.restart();
			}

			public void widgetDefaultSelected(SelectionEvent e) {
			}

		});

		(new Label(controlPanel, SWT.NONE)).setText("Charge:");

		final Text txtCharge = new Text(controlPanel, SWT.BORDER);

		Button btnSetCharge = new Button(controlPanel, SWT.PUSH);
		btnSetCharge.setText("Set");
		btnSetCharge.addSelectionListener(new SelectionListener() {
			public void widgetSelected(SelectionEvent e) {
				graph.setCharge(Float.parseFloat(txtCharge.getText()));
			}

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		(new Label(controlPanel, SWT.NONE)).setText("Link Distance:");

		final Text txtLinkDist = new Text(controlPanel, SWT.BORDER);

		Button btnLinkDist = new Button(controlPanel, SWT.PUSH);
		btnLinkDist.setText("Set");
		btnLinkDist.addSelectionListener(new SelectionListener() {
			public void widgetSelected(SelectionEvent e) {
				graph.setLinkDistance(Integer.parseInt(txtLinkDist.getText()));
			}

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		(new Label(controlPanel, SWT.NONE)).setText("Gravity:");

		final Text txtGravity = new Text(controlPanel, SWT.BORDER);

		Button btnGravity = new Button(controlPanel, SWT.PUSH);
		btnGravity.setText("Set");
		btnGravity.addSelectionListener(new SelectionListener() {
			public void widgetSelected(SelectionEvent e) {
				graph.setGravity(Float.parseFloat(txtGravity.getText()));
			}

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		Button btnPrev = new Button(controlPanel, SWT.PUSH);
		btnPrev.setText("< Prev");
		btnPrev.addSelectionListener(new SelectionListener() {
			public void widgetSelected(SelectionEvent e) {
						if (dbIndex == 0) 
							return;
						dbIndex--;
						final MLNDB db = dbs.get(dbIndex);
						graph.removeAllNodes();
						id2node.clear();
						// nodes2link.clear();
						for (String dom : db.domains.keySet()) {
							for (String value : db.domains.get(dom)) {
								String id = "" + dbCounter + dom + value;
								if (!id2node.containsKey(id)) {
									Node n = new Node(value, "#dddddd", "");
									id2node.put(id, n);
									graph.addNode(n);
								}
							}
						}

						for (Atom a : db.getAtoms()) {
							assert (a.args.length == 2);
							String dom1 = db.predicates.get(a.predName)[0];
							String dom2 = db.predicates.get(a.predName)[1];
							final String id1 = dbCounter + dom1 + a.args[0];
							final String id2 = dbCounter + dom2 + a.args[1];
							System.out.println(id1);
							System.out.println(id2);
							final Atom atom = a;
							graph.addLink(id2node.get(id1), id2node.get(id2), atom.predName, true, db.get(atom));
						}
					}

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});


		Button btnNext = new Button(controlPanel, SWT.PUSH);
		btnNext.setText("Next >");
		final Display display = Display.getCurrent();
		btnNext.addSelectionListener(new SelectionListener() {
			public void widgetSelected(SelectionEvent e) {

//				(new Thread() {
//					public void run() {
						if (dbIndex == dbs.size()) 
							return;
						dbIndex++;
						final MLNDB db = dbs.get(dbIndex);
						graph.removeAllNodes();
						id2node.clear();
						// nodes2link.clear();
						for (String dom : db.domains.keySet()) {
							for (String value : db.domains.get(dom)) {
								String id = "" + dbCounter + dom + value;
								if (!id2node.containsKey(id)) {
									Node n = new Node(value, "#dddddd", "");
									id2node.put(id, n);
									graph.addNode(n);
								}
							}
						}

						for (Atom a : db.getAtoms()) {
							assert (a.args.length == 2);
							String dom1 = db.predicates.get(a.predName)[0];
							String dom2 = db.predicates.get(a.predName)[1];
							final String id1 = dbCounter + dom1 + a.args[0];
							final String id2 = dbCounter + dom2 + a.args[1];
							System.out.println(id1);
							System.out.println(id2);
							final Atom atom = a;
							graph.addLink(id2node.get(id1), id2node.get(id2), atom.predName, true, db.get(atom));
//							display.asyncExec(new Runnable() {
//								public void run() {
//									main.graph.addLink(newNode, prevNode, "", false, 1f);
								}
//							});
						}
//						try {
//							Thread.sleep(300);
//						} catch (InterruptedException e) {
//							// TODO Auto-generated catch block
//							e.printStackTrace();
//						}
//					}
//				}).start();
			//}
			

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});

		(new Label(controlPanel, SWT.NONE)).setText("Instruction :");

		final Text txtInstruction = new Text(controlPanel, SWT.BORDER);

		Button btnInstruction = new Button(controlPanel, SWT.PUSH);
		btnInstruction.setText("Parse");
		btnInstruction.addSelectionListener(new SelectionListener() {
			public void widgetSelected(SelectionEvent e) {
				graph.removeAllNodes();
				id2node.clear();
				String instruction = txtInstruction.getText();
				System.out.println("instruction " + instruction);
				queryPracRos(instruction);
			}

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});
		
		
		final ServerPushSession pushSession = new ServerPushSession();
		pushSession.start();
		// Start the ROS node
		new Thread(new Runnable() {
			public void run() {
				try {
					Ros ros = Ros.getInstance();
					ros.init("pracrap");
					NodeHandle n = ros.createNodeHandle();

					Subscriber<MLNDatabase> sub;
					sub = n.subscribe("/rosprac/pracinfer", new MLNDatabase(), new ROSPRAC(display, PRACMain.this), 10);

					n.spin();
					n.shutdown();
				} catch (RosException e1) {
					e1.printStackTrace();
				}
			}
		}).start();

		graph.setLinkDistance(150);
		graph.setCharge(-3000);
	}

	// private void setChargeDialog() {
	// String title = "Settings";
	// String mesg = "Set new charge of nodes:";
	// String def = "-100";
	// final InputDialog dialog = new InputDialog(getShell(), title, mesg, def,
	// null) {
	//
	// @Override
	// public boolean close() {
	// boolean result = super.close();
	// // int returnCode = dialog.getReturnCode();
	// graph.setCharge(Float.parseFloat(getValue()));
	// return result;
	// }
	// };
	// dialog.open();
	// }

	// public static void runPython() {
	// // String prac = "/home/ai/Documents/Aptana_Studio_3_Workspace/prac/";
	// String pracPath = System.getenv("PRAC_HOME");
	// PythonInterpreter python = new PythonInterpreter();
	//
	// python.exec("import sys");
	// PySystemState sys = Py.getSystemState();
	// sys.path.append(new PyString("/usr/lib/python2.7/dist-packages"));
	// String pythonPath = System.getenv("PYTHONPATH");
	// String[] pythonPaths = pythonPath.split(File.pathSeparator);
	// for (String p : pythonPaths)
	// sys.path.append(new PyString(p));
	//
	// python.exec("from mln.database import readDBFromString");
	// System.out.println(sys.path);
	// // sys.path.append(new PyString(pracPath + "pracmodules"));
	// sys.path.append(new PyString(pracPath + "/praccore"));
	// // python.exec("from prac.core import PRAC");
	//
	// // try to load PRAC
	// // python.exec("prac = PRAC()");
	// // PyObject prac = python.get("prac");
	// // try to load an MLN datase
	// python.exec("db = readDBFromString('''dobj(Flip-1, pancake-3)''')");
	// PyObject db = python.get("db");
	//
	// }

	public int showMessageBox(String title, String message, int style) {
		MessageBox messageBox = new MessageBox(getShell(), style);
		messageBox.setText(title);
		messageBox.setMessage(message);
		PRACDialogUtils.ReturnCodeDlgCallback callback = new PRACDialogUtils.ReturnCodeDlgCallback();
		DialogUtil.open(messageBox, callback);
		return callback.getReturnCode();
	}
	
	/**
	 * Method for querying the ros service that performs the prac inference
	 */
	
	public void queryPracRos(String instruction) {
		// initialize the list of prac modules to be called
		initPracModules();
		Ros ros = Ros.getInstance();
		NodeHandle n = ros.createNodeHandle();
		ServiceClient<PRACInfer.Request, PRACInfer.Response, PRACInfer> sc =
				          n.serviceClient(PRAC_INFER , new PRACInfer(), false);
		
		PRACInfer.Request rq = new PRACInfer.Request();
		Response response = null;
		ArrayList<String> instructions = new ArrayList<String>();
		instructions.add(instruction);
		 
		dbs.clear();
		try {
			ROSPRAC rosprac = new ROSPRAC(Display.getCurrent(), this);
			// call different modules of prac in the order that is set by initPracModules()
			for (String module : pracModules) {
				rq.instructions = instructions;
				rq.pracmodule = module;
				if (response != null) {
					rq.input_dbs = response.output_dbs;
					rq.instructions.clear();
				}
				response = sc.call(rq);
				for (MLNDatabase dbmsg : response.output_dbs) {
					rosprac.call(dbmsg);
				}

			}
		}
		catch(RosException rex){
			rex.printStackTrace();
			System.exit(ROS_EXCEPTION);
		}	
	}

	// public static void main(String[] args) {
	// runPython();
	// }
}

