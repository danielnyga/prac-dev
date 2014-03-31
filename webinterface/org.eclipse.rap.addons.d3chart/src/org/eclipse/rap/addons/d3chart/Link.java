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

public class Link {
	
	public Node source = null;
	public Node target = null;
	public boolean directed = false;
	public String label = "";
	public float strength = 1f;
	public int idx = -1;
	
	public Link(Node source, Node target, String label, boolean directed, float strength) {
		this.source = source;
		this.target = target;
		this.label = label;
		this.directed = directed;
		this.strength = strength;
	}

}
