'''
Created on Apr 21, 2012

@author: nyga
'''
import jpype
import platform

arch = {'x86_64': 'amd64', 'i686': 'i386'}
classpath = []

def startJvm():
    import os
    #print stanford_parser_home
    java_home = os.environ['JAVA_HOME']
    machine_arch = platform.machine()
#    print  machine_arch
    if machine_arch not in arch.keys():
        raise Exception('Your system architecture is not supported: %s' % machine_arch)
    java_home += '/jre/lib/%s/libjava.so' % arch[machine_arch]   
    #print java_home
    #print 'classpath=', classpath
    jpype.startJVM(jpype.getDefaultJVMPath(), '-ea', '-Djava.class.path=%s' % (':'.join(classpath)))
    print 'starting JVM...'
#    jpype.startJVM(java_home, '-ea', '-Djava.class.path=%s' % (':'.join(classpath)))
    print 'JVM running'
    
def shutdownJvm():
    jpype.shutdownJVM()