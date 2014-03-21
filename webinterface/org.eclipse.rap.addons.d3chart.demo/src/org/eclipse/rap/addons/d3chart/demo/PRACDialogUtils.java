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
package org.eclipse.rap.addons.d3chart.demo;

import org.eclipse.rap.rwt.widgets.DialogCallback;
import org.eclipse.swt.SWT;

public class PRACDialogUtils {
	public static class ReturnCodeDlgCallback implements DialogCallback {
		
		int returnCode = -1;
		
		public void dialogClosed(int returnCode) {
			this.returnCode = returnCode;
		}
		
		public int getReturnCode() {
			return this.returnCode;
		}
	}
}
