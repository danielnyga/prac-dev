"""
Created on Apr 21, 2012

@author: nyga
"""
import jpype
import platform
from pracmln.praclog import logger
import os


arch = {'x86_64': 'amd64', 'i686': 'i386'}
classpath = []

log = logger(__name__)


def startJvm():
    java_home = os.environ['JAVA_HOME']
    machine_arch = platform.machine()
    if machine_arch not in arch.keys():
        raise Exception('Your system architecture is not supported: {}'.format(
            machine_arch))
    java_home += '/jre/lib/{}/libjava.so'.format(arch[machine_arch])
    log.debug(jpype.getDefaultJVMPath())
    log.info('starting JVM...')
    jpype.startJVM(jpype.getDefaultJVMPath(), '-Xmx512m', '-Xms512m',
                   '-verbose:gc', '-ea',
                   '-Djava.class.path={}'.format(':'.join(classpath)))
    log.info('JVM running')


def shutdownJvm():
    jpype.shutdownJVM()


def isJvmRunning():
    return jpype.isJVMStarted()


def gc():
    jpype.java.lang.Runtime.getRuntime().gc()
    log.debug(jpype.java.lang.Runtime.getRuntime().freeMemory())
