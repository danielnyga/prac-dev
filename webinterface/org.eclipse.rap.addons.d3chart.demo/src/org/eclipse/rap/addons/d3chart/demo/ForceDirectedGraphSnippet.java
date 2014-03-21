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
package org.eclipse.rap.addons.d3chart.demo;

import org.eclipse.rap.addons.d3chart.ForceDirectedGraph;
import org.eclipse.rap.json.JsonObject;
import org.eclipse.rap.rwt.application.AbstractEntryPoint;
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
import org.eclipse.swt.widgets.MessageBox;
import org.python.core.Py;
import org.python.core.PyString;
import org.python.core.PySystemState;
import org.python.util.PythonInterpreter;

public class ForceDirectedGraphSnippet extends AbstractEntryPoint {

	private ForceDirectedGraph graph = null;

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
				showMessageBox("Hello, PRAC!", "Welcome to PRAC!", SWT.OK);
//				graph.showGraph();
//				graph.restart();
			}

			public void widgetDefaultSelected(SelectionEvent e) {
			}
		});
		
		Button btnAddNode = new Button(controlPanel, SWT.PUSH);
		btnAddNode.setText("Add a Node");
		btnAddNode.addSelectionListener(new SelectionListener() {
			
			public void widgetSelected(SelectionEvent e) {
				JsonObject Labarre = new JsonObject();
			    Labarre.add("name", "jeff");
			    Labarre.add("group", 2);
			    graph.addNode(Labarre);
//			    graph.restart();
			}
			
			public void widgetDefaultSelected(SelectionEvent e) {
				// TODO Auto-generated method stub
				
			}
		});
		
		
	}

	public void runPython() {
//	        String prac = "/home/ai/Documents/Aptana_Studio_3_Workspace/prac/";
		String pracPath = System.getenv("PRAC_HOME");
		PythonInterpreter interp = new PythonInterpreter();
		
		interp.exec("import sys");
		
		PySystemState sys = Py.getSystemState();
		sys.path.append(new PyString(pracPath + "pracmodules"));	        
		sys.path.append(new PyString(pracPath + "praccore"));	        
		
	}

	public int showMessageBox(String title, String message, int style) {
		MessageBox messageBox = new MessageBox(getShell(), style);
		messageBox.setText(title);
		messageBox.setMessage(message);
		PRACDialogUtils.ReturnCodeDlgCallback callback = new PRACDialogUtils.ReturnCodeDlgCallback();
		DialogUtil.open(messageBox, callback);
		return callback.getReturnCode();
	}
}
