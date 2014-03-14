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
import org.eclipse.rap.rwt.application.AbstractEntryPoint;
import org.eclipse.swt.SWT;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.widgets.Composite;


public class ForceDirectedGraphSnippet extends AbstractEntryPoint {

  @Override
  protected void createContents( Composite parent ) {
	parent.setLayout(new FillLayout());
	new ForceDirectedGraph(parent, SWT.NONE);
  }

}
