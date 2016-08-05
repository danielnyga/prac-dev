'''
Created on Apr 21, 2012

@author: nyga
'''
import os
import jpype
import platform
from pracmln import praclog


logger = praclog.logger(__name__, praclog.INFO)
arch = {'x86_64': 'amd64', 'i686': 'i386'}
classpath = []


def startJvm():
    java_home = os.environ['JAVA_HOME']
    machine_arch = platform.machine()
    if machine_arch not in arch.keys():
        raise Exception('Your system architecture is not supported: {}'.format(machine_arch))
    java_home += '/jre/lib/{}/libjava.so'.format(arch[machine_arch])
    logger.debug('starting JVM...')
    jpype.startJVM(jpype.getDefaultJVMPath(), '-Xmx1024m', '-Xms1024m',
                   '-verbose:gc', '-ea',
                   '-Djava.class.path={}'.format(':'.join(classpath)))
    logger.debug('JVM running')


def shutdownJvm():
    jpype.shutdownJVM()


def isJvmRunning():
    return jpype.isJVMStarted()


def gc():
    jpype.java.lang.Runtime.getRuntime().gc()
    logger.debug(jpype.java.lang.Runtime.getRuntime().freeMemory())
