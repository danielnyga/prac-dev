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
package prac.rap.main;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.eclipse.rap.addons.d3chart.Link;
import org.eclipse.rap.addons.d3chart.Node;
import org.eclipse.rap.rwt.RWT;
import org.eclipse.rap.rwt.client.Client;
import org.eclipse.rap.rwt.service.UISession;
import org.eclipse.swt.widgets.Display;

import prac.rap.data.MLNDB;
import prac.rap.data.MLNDB.Atom;
import ros.*;
import ros.communication.*;
import ros.pkg.rosmln.msg.MLNDatabase;
import ros.pkg.rosmln.msg.AtomProbPair;
//import ros.pkg.rosmln.msg.MLNInference;
//import ros.pkg.rosmln.msg.MRFClique;

public class ROSPRAC extends Subscriber.QueueingCallback<MLNDatabase> {

	
	Display display = null;
	PRACMain main = null;

	public ROSPRAC(Display display, PRACMain main) {
		this.display = display;
		this.main = main;
	}

//	public void call(final MLNInference inf) {
//		final List<MRFClique> cliques = inf.cliques;
//		ROSPRAC.this.display.asyncExec(new Runnable() {
//			public void run() {
//				main.graph.removeAllNodes();
//			}
//		});
//		id2node.clear();
//		nodes2link.clear();
//		
//		for (MRFClique c : cliques) {
//			List<Node> prev = new ArrayList<Node>();
//			System.out.println(c.variables);
//			for (String atom : c.variables) {
//				Node n = id2node.get(atom);
//				if (n == null) {
//					n = new Node(atom, "grey", "");
//					id2node.put(atom, n);
//					final Node newNode = n;
//					ROSPRAC.this.display.asyncExec(new Runnable() {
//						public void run() {
//							main.graph.addNode(newNode);
//						}
//					});
//				}
//				for (Node p : prev) {
//					if (nodes2link.containsKey(new String[] { p.label, n.label })
//							|| nodes2link.containsKey(new String[] { n.label, p.label }))
//						continue;
//					else {
//						final Node prevNode = p;
//						final Node newNode = n;
//						nodes2link.put(new String[] { prevNode.label, newNode.label }, null);
//						ROSPRAC.this.display.asyncExec(new Runnable() {
//							public void run() {
//								main.graph.addLink(newNode, prevNode, "", false, 1f);
//							}
//						});
//					}
//					try {
//						Thread.sleep(500);
//					} catch (InterruptedException e) {
//						// TODO Auto-generated catch block
//						e.printStackTrace();
//					}
//				}
//				prev.add(n);
//			}
//		}
//	}

	public void call(final MLNDatabase dbmsg) {

		MLNDB db;
		try {
			db = new MLNDB(new String[] { "dobj(word, word)", "det(word, word)", "has_sense(word, sense)",
					"is_a(sense, concept)", "action_role(word, role)", "action_core(word, actioncore)" });

			for (AtomProbPair a : dbmsg.evidence) {
				if (a.prob > 0 || a.atom.equals("is_a")) {
					db.put(a.atom, a.prob);
				}
			}
			main.dbs.add(db);
		} catch (Exception e1) {
			e1.printStackTrace();
		}
	}
}
