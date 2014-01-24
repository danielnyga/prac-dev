'''
Created on Aug 16, 2012

@author: meyer
'''
import os
import re
import subprocess
import shutil

class annotationFiles:
    def getAnnotationFiles(self, rowcount, wordcount, folder):
        dirList = os.listdir(folder)
        res = []
        for f in dirList:
            if f[0] == ".":
                continue
            elif f.startswith('Make'):
                if len(open(os.path.join(folder,f)).readlines())<rowcount:
                    tempF = open(os.path.join(folder,f))
                    maxWords = 0
                    for line in tempF.readlines():
                        if len(line.split(" ")) > maxWords:
                            maxWords = len(line.split(" "))
                    if maxWords <= wordcount:
                        res.append(os.path.join(folder,f))
	print res
        return res
    
    def createListFile(self, fileList, location):
        f = open(location, 'w')
        print 'created file: ' + location
        print "Filling File"
        for inFile in fileList:
            f.write(inFile + '\n')
        print str(len(fileList)) + " lines written."
        f.close()
    
    def subSelectFilesByLineNumber(self,file, lines, asList=False, outputFile=None):
	"""Get the content of a line in a file by line number."""

        f = open(file)
        res = []
        i=0
        for line in f.readlines():
            if i in lines:
                res.append(line.rsplit('.ehow')[0])
            i+=1
        f.close()
	if asList:
		return res

        self.createListFile(res, outputFile)
        
    def copyFilesWithNewEnding(self, filelist, sourcefolder, ending, destinationfolder):
	"""Copy all files provided from a list to a new location with a new ending."""
	print """CopyFilesWithNewEnding invoked with : sourcefolder = {sf}, 
			ending = {en}, destination folder = {dest}""".format(sf=sourcefolder, 
									en=ending, 
									dest =destinationfolder)
	source_folder = sourcefolder
	destination_folder = destinationfolder
			
	for f in filelist:
		file_name = f.rstrip()
		#print f
		#print file_name
		#file_path = os.path.join(source_folder, f)
		file_path = f.rstrip()
		output_path = os.path.join(destination_folder, f.rsplit('/',1)[1].rstrip() + '.' + ending)
		print "File Path ={p}".format(p=file_path)
		print "Output Path = {p}".format(p=output_path)
		shutil.copy(file_path, output_path)


if __name__ == "__main__":
    folder = os.path.join(os.getcwd(),'..','..','..','data','howtos','plain')
    output = os.path.join(os.getcwd(),'fileList.list')
    #out2 = os.path.join(os.getcwd(), 'sublist.list')
    af = annotationFiles()
    files = [88, 85, 76, 73, 68, 66, 53, 46, 45, 45, 44, 43, 38, 29, 27, 22, 20, 19, 18, 16, 14, 11, 10,7,6,5,2,0]
    res = af.subSelectFilesByLineNumber(output, files, asList=True)
    #print res
    af.copyFilesWithNewEnding(res , folder, "ehow", os.path.join(os.getcwd(), "ehow_files")) 
    #af.subSelectFilesByLineNumber(output, out2, files)
    #files = af.getAnnotationFiles(5, 10, folder)
    #af.createListFile(files, output)
    print "hallo"
