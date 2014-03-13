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
import org.eclipse.swt.internal.SWTEventListener;
import org.eclipse.swt.widgets.Canvas;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.Event;
import org.eclipse.swt.widgets.Listener;
import org.eclipse.rap.json.JsonObject;
import org.eclipse.swt.widgets.Item;

public class ForceDirectedGraph extends Canvas {
	
	private static final String REMOTE_TYPE = "d3graph.ForceDirectedGraph";
	private final RemoteObject remoteObject;
	private final List<Item> items;
	
	public ForceDirectedGraph( Composite parent, int style ) {
		    super( parent, style );
		    items = new LinkedList<Item>();
		    System.out.println("Hello there");
		    remoteObject = RWT.getUISession().getConnection().createRemoteObject( REMOTE_TYPE );
		    System.out.println("remote object has been set");
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
		    System.out.println("Done");
	}
}
