"""
Created on Apr 21, 2012

@author: nyga
"""
import jpype
import platform
import logging

arch = {'x86_64': 'amd64', 'i686': 'i386'}
classpath = []


def startJvm():
    import os
    java_home = os.environ['JAVA_HOME']
    machine_arch = platform.machine()
    if machine_arch not in arch.keys():
        raise Exception('Your system architecture is not supported: %s' % machine_arch)
    java_home += '/jre/lib/%s/libjava.so' % arch[machine_arch]   
    log = logging.getLogger('java')
    log.debug(jpype.getDefaultJVMPath())
    print 'starting JVM...'
    jpype.startJVM(jpype.getDefaultJVMPath(), '-Xmx512m', '-Xms512m', '-ea',
                   '-Djava.class.path=%s' % (':'.join(classpath)))
    print 'JVM running'


def shutdownJvm():
    jpype.shutdownJVM()


def isJvmRunning():
    return jpype.isJVMStarted()