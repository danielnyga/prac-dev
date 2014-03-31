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

import java.util.ArrayList;
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


public class ForceDirectedGraph extends Canvas {
	
	private static final String REMOTE_TYPE = "d3graph.ForceDirectedGraph";
	private final RemoteObject remoteObject;
	private final List<Item> items = new LinkedList<Item>();
	
	public List<Node> nodes = new ArrayList<Node>();
	public List<Link> links = new ArrayList<Link>();
	
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
	}
	
	public void addNode(Node node) {
		JsonObject json = new JsonObject();
		json.add("label", node.label);
		json.add("color", node.color);
		node.idx = this.nodes.size();
		remoteObject.call("addNode", json);
		nodes.add(node);
	}
	
	public void addLink(Link link) {
		JsonObject json = new JsonObject();
		json.add("label", link.label);
		json.add("source", link.source.idx);
		json.add("target", link.target.idx);
		json.add("directed", link.directed);
		json.add("strength", link.strength);
		link.idx = this.links.size();
		remoteObject.call("addLink", json);
		links.add(link);
	}
	
	public Link addLink(Node source, Node target, String label, boolean directed, float strength) {
		Link l = new Link(source, target, label, directed, strength);
		this.addLink(l);
		return l;
	}
	
	public void showGraph() {
		remoteObject.call("showGraph", new JsonObject());
	}
	
	public void restart() {
		remoteObject.call("restart", new JsonObject());
	}
	
	public void setCharge(float charge) {
		JsonObject params = new JsonObject();
		params.add("charge", charge);
		remoteObject.call("setCharge", params);
	}
	
	public void setLinkDistance(int distance) {
		JsonObject params = new JsonObject();
		params.add("linkdist", distance);
		remoteObject.call("setLinkDistance", params);
	}
	
	public void setFriction(int friction) {
		JsonObject params = new JsonObject();
		params.add("p", friction);
		remoteObject.call("setFriction", params);
	}
	
	public void setChargeDistance(int chargeDistance) {
		JsonObject params = new JsonObject();
		params.add("p", chargeDistance);
		remoteObject.call("setChargeDistance", params);
	}
	
	public void setGravity(float gravity) {
		JsonObject params = new JsonObject();
		params.add("p", gravity);
		remoteObject.call("setGravity", params);
	}
	
	public void removeAllNodes() {
		remoteObject.call("removeAllNodes", new JsonObject());
		this.nodes.clear();
		this.links.clear();
	}
	
}

