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
package prac.rap.data;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


/**
 * Represents a Database in MLNs.
 */
public class MLNDB {
	
	protected Map<String, Float> evidence = new HashMap<String, Float>();
	public Map<String, String[]> predicates = new HashMap<String, String[]>();
	public Map<String, String[]> domains = new HashMap<String, String[]>();
	
	/**
	 * Expects a list atom string specifying the predicate
	 * declarations, i.e. the predicate names and the domain names
	 * of their arguments.
	 * @throws Exception 
	 */
	public MLNDB(String[] predicates) throws Exception {
		for (String atom: predicates) {
			atom = atom.trim();
			if (atom.length() == 0)
				continue;
			Atom a = new Atom(atom);
			String[] argList = this.predicates.get(a.predName);
			if (argList != null && ! Arrays.equals(argList, a.args))
				throw new Exception("Inconsistent predicate decalarations: " + atom + " <-->" + new Atom(a.predName, argList));
			else
				this.predicates.put(a.predName, a.args);
		}
	}
	
	public Atom[] getAtoms() {
		Atom[] atoms = new Atom[evidence.size()];
		String[] atomStrings = new ArrayList<String>(this.evidence.keySet()).toArray(new String[0]);
		for (int i = 0; i < atoms.length; ++i)
			atoms[i] = new Atom(atomStrings[i]);
		return atoms;
	}
	
	public float get(String atom) {
		Atom a = new Atom(atom);
		return this.evidence.get(a.toString());
	}
	
	public float get(Atom atom) {
		return this.get(atom.toString());
	}
	
	public void put(Atom atom, float truth) {
		String[] domainNames = this.predicates.get(atom.predName);
		assert (domainNames != null);
		for (int i = 0; i < domainNames.length; ++i) {
			String domName = domainNames[i];
			String[] domain = this.domains.get(domName);
			if (domain == null)
				domain = new String[0];
			List<String> values = new ArrayList<String>(Arrays.asList(domain));
			System.out.println(values);
			if (! values.contains(atom.args[i])) {
				System.out.println(atom.args[i]);
				values.add(atom.args[i]);
				this.domains.put(domName, values.toArray(new String[0]));
			}
		}
		this.evidence.put(atom.toString(), truth);
	}
	
	public void put(Atom atom) {
		this.put(atom, 1f);
	}
	
	public void put(String atom, float truth) {
		this.put(new Atom(atom), truth);
	}
	
	public void put(String atom) {
		this.put(new Atom(atom));
	}
	
	public void printEvidence() {
		for (String a: this.evidence.keySet())
			System.out.println(String.format("%.3f\t%s", this.get(a), a));
	}
	
	
	/**
	 * Represents an atom in an MLN Database.
	 */
	public static class Atom {
		
		public String predName;
		public String[] args;
		
		public Atom(String atom) {
			// Parse the atom
			int bracket1 = atom.indexOf('(');
			int bracket2 = atom.indexOf(')', bracket1);
			this.predName = atom.substring(0, bracket1).trim();
			String argsStr = atom.substring(bracket1 + 1, bracket2);
			this.args = argsStr.split(",");
			for (int i = 0; i < args.length; ++i)
				this.args[i] = args[i].trim();
		}
		
		public Atom(String predName, String[] args) {
			this.predName = predName;
			this.args = args;
		}
		
		public String toString() {
			String argsStr = "";
			for (int i = 0; i < this.args.length; ++i) {
				argsStr += this.args[i];
				if (i < this.args.length - 1)
					argsStr += ",";
			}
			return this.predName + "(" + argsStr + ")";
		}
		
	}
	
	public static void main(String[] args) {
		MLNDB db;
		try {
			db = new MLNDB(new String[] {"pred1(p1, p2, p3)", "pred2(numbers, numbers, numbers)"});
			String dbString = "pred1(a1,a2,a3)\n"
					+ "pred2(4,5,6)";
			String[] lines = dbString.split("\n");
			for (String l: lines)
				db.put(l);
			db.printEvidence();
			
			for (String pred: db.predicates.keySet())
				System.out.println(Arrays.toString(db.predicates.get(pred)));
			
			for (String dom: db.domains.keySet())
				System.out.println(Arrays.toString(db.domains.get(dom)));
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}

}
