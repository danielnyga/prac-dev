#!/usr/bin/python
"""
Author: Jeremy M. Stober
Program: MDS.PY
Description: Multidimensional Scaling
"""

import os, sys, getopt, pdb
from numpy import *
from numpy.linalg import *
from numpy.random import *
import pylab
from mln.util import mergeDomains
from praclog import logging
from prac.core import PRAC
from mln.database import readAllDBsInDir, readDBFromString, readDBFromFile
from prac_nltk.corpus import wordnet as wn
from matplotlib.pyplot import text
from scipy.optimize.optimize import fmin_bfgs


def doMDS(d, dimensions = 2):
    """
    Multidimensional Scaling - Given a matrix of interpoint distances,
    find a set of low dimensional points that have similar interpoint
    distances.
    """

    (n,n) = d.shape
    E = (-0.5 * d**2)

    # Use mat to get column and row means to act as column and row means.
    Er = mat(mean(E,1))
    Es = mat(mean(E,0))

    # From Principles of Multivariate Analysis: A User's Perspective (page 107).
    F = array(E - transpose(Er) - Es + mean(E))

    [U, S, V] = svd(F)

    Y = U * sqrt(S)

    return (Y[:,0:dimensions], S)

def norm(vec):
    return sqrt(sum(vec**2))

def square_points(size):
    nsensors = size ** 2
    return array([(i / size, i % size) for i in range(nsensors)])

def test():

    points = square_points(10)
    print points

    distance = zeros((100,100))
    for (i, pointi) in enumerate(points):
        for (j, pointj) in enumerate(points):
            distance[i,j] = norm(pointi - pointj)
    print distance
    Y, eigs = doMDS(distance)

    pylab.figure(1)
    pylab.plot(Y[:,0],Y[:,1],'.')

    pylab.figure(2)
    pylab.plot(points[:,0], points[:,1], '.')

    pylab.show()


def show_clusters_of_concepts(*dbs):
    '''
    Takes a sequence of databases and performs a multi-dimensional scaling on
    the synsets given by the merged set of 'concept' domains in them.
    '''
    log = logging.getLogger(__name__)
    domains = mergeDomains(*map(lambda d: d.domains, dbs))
    concepts = domains.get('concept', None)
    if concepts is None:
        log.error('Domain "concepts" not found in databases.')
        return
    if 'null' in concepts: # remove the null concept
        del concepts[concepts.index('null')]
    synsets = map(lambda x: wn.synset(x), concepts)  # @UndefinedVariable
    distance = zeros((len(synsets),len(synsets)))
    for (i, pointi) in enumerate(synsets):
        for (j, pointj) in enumerate(synsets):
            sys.stdout.write('%d / %d      \r' % (i, len(synsets)))
            sim = synsets[i].path_similarity(synsets[j])
            if sim is None: sim = 0
            distance[i,j] = 1. - sim
    Y, eig = doMDS(distance, dimensions=2)
    pylab.figure(1)
    for i, s in enumerate(synsets):
        text(Y[i,0],Y[i,1], s.name, fontsize=8)
#         text(0, Y[i,0], s.name, fontsize=8)
    pylab.plot(Y[:,0],Y[:,1],'.')
#     pylab.plot([0] * len(distance),Y[:,0],'.')
    pylab.show()
    

if __name__ == '__main__':
    prac = PRAC()
    dbs = list(readDBFromFile(prac.mln, os.path.join('/', 'home', 'nyga', 'work', 'nl_corpora', 'wikihow', 'Slicing.db')))
    domains = mergeDomains(*map(lambda d: d.domains, dbs))
    concepts = domains.get('concept', None)
    print len(concepts)
    print concepts
#     show_clusters_of_concepts(*dbs[:len(dbs)])
#     show_clusters_of_concepts(readDBFromString(prac.mln, 'concept = {spatula.n.01, pancake.n.01, milk.n.01}'))
    
   
