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

public class Node {

	public String color = "#DDDDDD";
	public String label = "";
	public int idx = -1;
	public String tooltip = "";
	
	public Node(String label, String color, String tooltip) {
		this.color = color;
		this.label = label;
		this.tooltip = tooltip;
	}
	
}
