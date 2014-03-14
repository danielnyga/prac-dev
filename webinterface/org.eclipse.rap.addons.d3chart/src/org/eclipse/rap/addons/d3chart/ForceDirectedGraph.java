/*******************************************************************************
 * Copyright (c) 2014 EclipseSource and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    Ralf Sternberg - initial API and implementation
 ******************************************************************************/
package org.eclipse.rap.addons.d3chart;

import static org.eclipse.rap.rwt.lifecycle.WidgetUtil.getId;

import java.util.LinkedList;
import java.util.List;

import org.eclipse.rap.rwt.RWT;
import org.eclipse.rap.rwt.remote.AbstractOperationHandler;
import org.eclipse.rap.rwt.remote.RemoteObject;
import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.rap.json.JsonObject;
import org.eclipse.swt.widgets.Item;

import org.python.core.Py;
import org.python.core.PyString;
import org.python.core.PySystemState;
import org.python.util.PythonInterpreter;


public class ForceDirectedGraph extends Canvas {
	
	private static final String REMOTE_TYPE = "d3graph.ForceDirectedGraph";
	private final RemoteObject remoteObject;
	private final List<Item> items = new LinkedList<Item>();

	public ForceDirectedGraph( Composite parent, int style ) {
		    super( parent, style );

		    remoteObject = RWT.getUISession().getConnection().createRemoteObject( REMOTE_TYPE );
		    remoteObject.set( "parent",getId( this ) );		    
		    remoteObject.setHandler( new AbstractOperationHandler() {
		      @Override
		      public void handleNotify( String eventName, JsonObject properties ) {
		        if( "Selection".equals( eventName ) ) {
		          Event event = new Event();
		          event.index = properties.get( "index" ).asInt();
		          event.item = items.get( event.index );
		          notifyListeners( SWT.Selection, event );
		        }
		      }
		    } );
		    
		    ChartResources.ensureJavaScriptResources();
		    
			JsonObject oldMan = new JsonObject();
		    oldMan.add("name", "jeff");
		    oldMan.add("group", 1);		    
		    this.addNode(oldMan);
		    
		    JsonObject Labarre = new JsonObject();
		    Labarre.add("name", "jeff");
		    Labarre.add("group", 2);		    
		    this.addNode(Labarre);

		    JsonObject Fauchelevent = new JsonObject();
		    Fauchelevent.add("name", "jeff");
		    Fauchelevent.add("group", 0);		    
		    this.addNode(Fauchelevent);
		    
		    JsonObject oneToZero = new JsonObject();
		    oneToZero.add("source", 1);
		    oneToZero.add("target", 0);	
		    oneToZero.add("value", 1);	
		    this.addLink(oneToZero);
		    
		    JsonObject twoToZero = new JsonObject();
		    twoToZero.add("source", 2);
		    twoToZero.add("target", 0);	
		    twoToZero.add("value", 1);	
		    this.addLink(twoToZero);
		    
		    runPython();
	}
	
	public void runPython() {
        String prac = "/home/ai/Documents/Aptana_Studio_3_Workspace/prac/";

	    PythonInterpreter interp = new PythonInterpreter();
	    interp.exec("import sys");
	    
        PySystemState sys = Py.getSystemState();
        sys.path.append(new PyString(prac + "pracmodules"));	        
        sys.path.append(new PyString(prac + "praccore"));	        

	}
	
	public void addNode(JsonObject jsonNode) {
		remoteObject.call("addNode", jsonNode);
	}
	
	public void addLink(JsonObject jsonLink) {
		remoteObject.call("addLink", jsonLink);
	}
}

