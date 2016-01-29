import traceback

import os
import time
import re
import sys
from nltk.corpus import wordnet as wn, WordNetCorpusReader
from prac.wordnet import WordNet
from random import shuffle

from mln import readMLNFromFile
from mln.database import Database,readDBFromFile
from multiprocessing import Pool, cpu_count
from nltk.stem import WordNetLemmatizer
from utils.eval import ConfusionMatrix

import xml.etree.cElementTree as ET
import math

from subprocess import Popen, PIPE, STDOUT

ims_path = "ims_0.9.2.1"
wnl = WordNetLemmatizer()
MLN = None
regex_new_db = re.compile("\n\s*-+\s*\n")
wordnet = WordNet(None)
cm_path_list = []

nounTags = ['NN', 'NNS', 'CD']
verbTags = ['VB', 'VBG', 'VBZ', 'VBD', 'VBN', 'VBP', 'MD']
adjTags = ['JJ', 'JJR', 'JJS']
posMap = {}
for n in nounTags:
    posMap[n] = 'n'
for v in verbTags:
    posMap[v] = 'v'
for a in adjTags:
    posMap[a] = 'a'

wn_171 = WordNetCorpusReader('/home/seba/Downloads/WordNet-1.7.1/dict', None)


offsets_noun_dict = dict([(s.offset(), s) for s in list(wn_171.all_synsets('n'))])
offsets_verb_dict = dict([(s.offset(), s) for s in list(wn_171.all_synsets('v'))])
offsets_adj_dict = dict([(s.offset(), s) for s in list(wn_171.all_synsets('a'))])

class XValFoldParams(object):
    
    def __init__(self, **params):
        self.pdict = params
        self.foldIdx = None
        self.foldCount = None
        self.learnDBs = None
        self.testDBs = None
        
class XValFold(object):
    '''
    Class representing and providing methods for a cross validation fold.
    '''
    
    def __init__(self, params):
        '''
        params being a XValFoldParams object.  
        '''
        self.params = params
        self.fold_id = 'Fold-%d' % params.foldIdx
        self.confMatrix = ConfusionMatrix()
            
    def run(self):
        '''
        Runs the respective fold of the crossvalidation.
        '''
        
        print 'Running fold %d of %d...' % (self.params.foldIdx + 1, self.params.foldCount)
        directory = os.path.join(self.params.directory,str(self.params.foldIdx))
        os.mkdir(directory)
        os.mkdir(os.path.join(directory,'model'))
        
        try:
                
            learnDBs_ = self.params.learnDBs
            testDBs_ = self.params.testDBs

            # train
            print 'Starting learning...'
            
            # store the learned MLN in a file
            create_training_set(learnDBs_, directory)
            print 'Finished training set'
            create_test_set(testDBs_,directory )
            print 'Finished test set'
            
            train_classifier(os.path.abspath(os.path.join(directory,'train.xml')), 
                             os.path.abspath(os.path.join(directory,'train.key')), 
                             os.path.abspath(os.path.join(directory,'model')))
            
            test_classifier(os.path.abspath(os.path.join(directory,'test.txt')), 
                                            os.path.abspath(os.path.join(directory,'model')),
                                            os.path.abspath(os.path.join(directory,'result.txt')))
            
           
            #cm = ConfusionMatrix()
            #self.evalMLN(learnedMLN, testDBs_, 'FirstOrderLogic',cm)
            #cm.toFile(os.path.join(directory,'FOL', 'conf_matrix_%d.cm' % self.params.foldIdx))
            
            print 'Evaluation finished.'
        
            cm = compare_results(testDBs_, os.path.abspath(os.path.join(directory,'result.txt')))
            cm.toFile(os.path.abspath(os.path.join(directory,'result.cm')))
            
            return cm
        except (KeyboardInterrupt, SystemExit):
            return None
    
class IMSDatabase(object):
    
    def __init__(self,sentence,db):
        self.sentence = sentence
        self.db = db

def get_wn30_to_wn171_dict(file_path):
    print 'Start process'
    mapping_file = open(file_path,'r')
    mapping_list = mapping_file.read().split('\n')
    mapping_dict = {}
    length_of_list = str(len(mapping_list))
    print length_of_list
    
    for mapping in mapping_list:
        mapping_splited = re.split("\s+",mapping)
        
        if len(mapping_splited) >= 2:
            wn30_offset = mapping_splited[0]
            wn17_offset = mapping_splited[1]
            mapping_dict[wn30_offset] = wn17_offset
    
    
    print "Finished calculating wn30_to_wn171_dict"
    mapping_file.close()
    print len(mapping_dict.keys())
    
    return mapping_dict


def get_offset_of_wn30_synset(sense):
    mapping_dict = None
    sense_synset = wordnet.synset(sense)
    syn_pos = sense_synset.pos()

    
    if syn_pos == 'n':
        mapping_dict = mapping_noun_dict
    
    if syn_pos == 'v':
        mapping_dict = mapping_verb_dict
    
    if syn_pos == 'a':
        mapping_dict = mapping_adj_dict
    
    if mapping_dict is None:
        return None
    
    offset = str(sense_synset.offset())
    
    if len(offset) < 8:
        for x in range(0,8-len(offset)):
            offset = "0"+offset
    
    if offset not in mapping_dict.keys():
        print "{} cannot be mapped".format(sense)
    else:
        offset = mapping_dict[offset]
    return int(offset)

def runFold(fold):
    try:
        return fold.run()
    except:
        print traceback.format_exc()
    return fold

def transform_30_nltk_sense_to_17_nltk_sense(sense):
    syn_pos = wn.synset(sense).pos()
    offset = get_offset_of_wn30_synset(sense)
    
    syn = None
    
    if syn_pos == 'n':
        syn = offsets_noun_dict[offset]
    if syn_pos == 'v':
        syn = offsets_verb_dict[offset]
    if syn_pos == 'a':
        syn = offsets_adj_dict[offset]
    
    if syn is None:
        return None
    
    return syn 

def transform_17_nltk_sense_to_ims_sense(sense):
    
    syn_pos = wn_171.synset(sense).pos()
    offset = wn_171.synset(sense).offset()
    
    syn = None
    
    if syn_pos == 'n':
        syn = offsets_noun_dict[offset]
    if syn_pos == 'v':
        syn = offsets_verb_dict[offset]
    if syn_pos == 'a':
        syn = offsets_adj_dict[offset]
    
    if syn is None:
        return None
    
    lemma = syn.name().split('.')[0]
    
    return get_index_sense(lemma, offset)
    
def transform_30_nltk_sense_to_ims_sense(sense):
    syn_pos = wn.synset(sense).pos()
    offset = get_offset_of_wn30_synset(sense)
    
    syn = None
    
    if syn_pos == 'n':
        syn = offsets_noun_dict[offset]
    if syn_pos == 'v':
        syn = offsets_verb_dict[offset]
    if syn_pos == 'a':
        syn = offsets_adj_dict[offset]
    
    if syn is None:
        return None
    
    lemma = syn.name().split('.')[0]
    
    return get_index_sense(lemma, offset)
    
def get_index_sense(lemma,offset):
    for key in wordnet_index_sense.keys():
        if (lemma in key) and (int(wordnet_index_sense[key]) == offset):
            sense = key
            break
        
    return sense
            
def get_wordnet_index_sense():
    index_file = open("index.sense",'r')
    sense_list = index_file.read().split('\n')
    pos_regex = re.compile('(.+)\s+(\d+)')
    
    wordnet_index_sense = {}
    for line in sense_list:
        res = pos_regex.search(line)
        if res:
            splited_res = re.split("\s+",res.group())
            sense_id = splited_res[0]
            offset = splited_res[1]
            wordnet_index_sense[sense_id] = offset
    
    return wordnet_index_sense

def train_classifier(xml_file,key_file,output_folder):
    cmd = './train_one.bash {} {} {}'.format(xml_file,key_file,output_folder)
    p = Popen(cmd, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    print p.stdout.read()

def test_classifier(plain_text_file,model_path,output_file):
    cmd = './testPlain.bash {} {} {} index.sense 0 0 0 0'.format(model_path,plain_text_file,output_file)
    p = Popen(cmd, shell=True, stdin=PIPE, stdout=PIPE, stderr=STDOUT, close_fds=True)
    print p.stdout.read()

def get_highest_probable_senses(result_file_path):
    regex_word_sense = re.compile('<x[^<]+')
    regex_sense = re.compile('[\w_%:]+\|\d+\.*\d*')
    result_file = open(result_file_path,'r')
    content = result_file.read().split('\n')[:-1]
    result_dbs_senses_list = []
    
    for c in content:
        senses = []
        regex_findall_word_sense_result = regex_word_sense.findall(c)
        for element in regex_findall_word_sense_result:
            #print 'ELEMENT {}'.format(element)
            regex_findall_result = regex_sense.findall(element)
            #print "Regex_findall_result {}".format(regex_findall_result)
            
            if len(regex_findall_result) == 1:
                e = regex_findall_result[0]
                senses.append(re.split("\|",e)[0])
             #   print "One Sense only {}".format(e)
            else:
                senses_dict = {}
                for e in regex_findall_result:
                    temp = re.split("\|",e)
                    senses_dict[float(temp[1])] = temp[0]
                
                probable_sense = sorted(senses_dict.keys(),reverse=True)[0]
                senses.append(probable_sense)
              #  print "DICT {}".format(senses_dict)
               # print "PROB {}".format(probable_sense)
        #print     
        result_dbs_senses_list.append(senses)
    
    return result_dbs_senses_list

def compare_results(test_dbs,result_file_path):
    cm = ConfusionMatrix()
    
    regex_word_sense = re.compile('<x[^<]+')
    regex_sense = re.compile('[\w_%:]+\|\d+\.*\d*')
    result_file = open(result_file_path,'r')
    content = result_file.read().split('\n')[:-1]
    #print content
    del_index_list = []
    result_dbs_senses_list = []
    test_dbs_senses_list = []
    num_of_preps_list = []
    for i  in range(0,len(test_dbs)):
        db = test_dbs[i].db
        num_of_preps = 0
        sense_list = []
        for q1 in db.query('predicate(?w)'):
            for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                sense_list.append(transform_30_nltk_sense_to_ims_sense(q2['?s']))

        for q1 in db.query('dobj(?w1,?w2)'):
            
            for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                sense_list.append(transform_30_nltk_sense_to_ims_sense(q2['?s']))

        for prep_pred in ['prep_into','prep_to','prep_with','prep_in','prep_on']:
            for q1 in db.query('{}(?w1,?w2)'.format(prep_pred)):
                for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    num_of_preps += 1
                    sense_list.append(transform_30_nltk_sense_to_ims_sense(q2['?s']))
        num_of_preps_list.append(num_of_preps)
        test_dbs_senses_list.append(sense_list)

    
    for i in range(0,len(content)):
        if i not in del_index_list:
            c = content[i]
            senses = []
            regex_findall_word_sense_result = regex_word_sense.findall(c)
            for element in regex_findall_word_sense_result:
                #print 'ELEMENT {}'.format(element)
                regex_findall_result = regex_sense.findall(element)
                #print "Regex_findall_result {}".format(regex_findall_result)
                
                if len(regex_findall_result) == 1:
                    e = regex_findall_result[0]
                    senses.append(re.split("\|",e)[0])
                 #   print "One Sense only {}".format(e)
                else:
                    senses_dict = {}
                    for e in regex_findall_result:
                        temp = re.split("\|",e)
                        senses_dict[float(temp[1])] = temp[0]
                    
                    probable_sense = sorted(senses_dict.keys(),reverse=True)[0]
                    senses.append(senses_dict[probable_sense])
                  #  print "DICT {}".format(senses_dict)
                   # print "PROB {}".format(probable_sense)
            #print     
            result_dbs_senses_list.append(senses)
        
    
    if result_dbs_senses_list :
        print result_dbs_senses_list
        for i in range(0,len(result_dbs_senses_list)):
            test_db = test_dbs_senses_list[i]
            result_db = result_dbs_senses_list[i]
            #Check predicates
            cm.addClassificationResult(test_db[0],result_db[0])
            if num_of_preps_list[i] > 0:
                cm.addClassificationResult(test_db[-1],result_db[-1])
                result_db = sorted(result_db[1:-1])
                test_db = sorted(test_db[1:-1])
            
            else:
                result_db = sorted(result_db[1:])
                test_db = sorted(test_db[1:])

            for j in range(0,len(result_db)):
                 print result_db[j]
                 print test_db[j]
                 print "XXXXXXXXXXXXx"
                 if result_db[j] in test_db:
                     cm.addClassificationResult(result_db[j],result_db[j])
                 else:
                     cm.addClassificationResult(test_db[j],result_db[j])
            
    
    return cm
    
def create_test_set(dbs,result_path):
    test_file = open(os.path.join(result_path,'test.txt'),"w")
    
    
    for db in dbs :
        test_file.write("{}\n".format(db.sentence))
    
    test_file.close()
                
def create_training_set(dbs,result_path):
    key_file = open(os.path.join(result_path,'train.key'),"w")
    
    corpus = ET.Element("corpus",lang='english')
    predicate_lemma_list = {}
    dobj_lemma_list = {}
    i = 0
    
    for db_ in dbs:
        db = db_.db
        predicate = ""
        dobj = []
        prepobj = ""
        prep_kind = ""
        prep_sense = ""

        pos = ""
        predicate_sense = ""
        dobj_sense = []
        
        i += 1
        
        for q1 in db.query('predicate(?w)'):
            predicate = wnl.lemmatize(q1['?w'],'v').lower()
            for q2 in db.query('has_sense({},?s)'.format(q1['?w'])):
                predicate_sense = transform_30_nltk_sense_to_ims_sense(q2['?s'])
        
        for q1 in db.query('dobj(?w1,?w2)'):
            for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    syn = wn.synset(q2['?s'])
                    word = syn.name().split(".")[0]
                    pos = str(syn.pos())
                    
                    if pos is not None:
                        dobj.append(wnl.lemmatize(word,pos))
                        
                    offset = get_offset_of_wn30_synset(q2['?s'])
                    if offset is not None:
                        dobj_sense.append(transform_30_nltk_sense_to_ims_sense(q2['?s']))
        
        for prep_pred in ['prep_into','prep_to','prep_with','prep_in','prep_on']:
            for q1 in db.query('{}(?w1,?w2)'.format(prep_pred)):
                for q2 in db.query('has_sense({},?s)'.format(q1['?w2'])):
                    prep_kind = prep_pred.split('_')[1]
                    syn = wn.synset(q2['?s'])
                    word = syn.name().split(".")[0]
                    pos = str(syn.pos())
                    
                    if pos is not None:
                        prepobj = wnl.lemmatize(word,pos)
                        
                    offset = get_offset_of_wn30_synset(q2['?s'])
                    if offset is not None:
                        prep_sense = transform_30_nltk_sense_to_ims_sense(q2['?s'])
        
        if dobj_sense:
            sentence = ""
            predicate_lemma = predicate + '.v'
            dobj_lemma = map(lambda x: x + '.n',dobj)
             
            if not prep_sense:
                sentence = '<head>' + predicate+ '</head> '+ ' and '.join(map(lambda x: "the "+x,dobj))+'.\n' 
            elif prep_kind == 'into':
                sentence = '<head>' + predicate+ '</head> '+ ' and '.join(map(lambda x: "the "+x,dobj))+' '+prep_kind +' ' + prepobj+'.\n' 
            else:
                sentence = '<head>' + predicate+ '</head> '+ ' and '.join(map(lambda x: "the "+x,dobj))+' '+prep_kind +' a ' + prepobj+'.\n' 

            if predicate_lemma not in predicate_lemma_list.keys():
                predicate_lemma_list[predicate_lemma] = [[sentence,predicate_sense]]
            else:
                predicate_lemma_list[predicate_lemma].append([sentence,predicate_sense])

            if prep_sense:
               lemma_key = prepobj+'.n' 

               if prep_kind == 'into':
                   sentence = predicate+' '+' and '.join(map(lambda x: "the "+x,dobj))+' '+prep_kind +'<head>' + prepobj+'</head>.\n' 
               else:
                   sentence = predicate+' '+ ' and '.join(map(lambda x: "the "+x,dobj))+' '+prep_kind +' a <head>' + prepobj+'</head>.\n' 

               if lemma_key not in dobj_lemma_list.keys():
                    dobj_lemma_list[lemma_key] = [[sentence,prep_sense]]
               else:
                    dobj_lemma_list[lemma_key].append([sentence,prep_sense])
                
            
            
            if len(dobj) == 1:
                element = dobj[0]
                if not prep_sense:
                    sentence = predicate+ ' the <head>'+ element+'</head>.\n' 
                elif prep_kind == 'into':
                    sentence = predicate+ ' the <head>'+ element+'</head>'+' ' +prep_kind +' ' + prepobj+'.\n' 
                else:
                    sentence = predicate+ ' the <head>'+ element+'</head>'+' ' +prep_kind +' a ' + prepobj+'.\n' 

                
                lemma_key = dobj_lemma[0] 
                if lemma_key not in dobj_lemma_list.keys():
                    dobj_lemma_list[lemma_key] = [[sentence,dobj_sense[0]]]
                else:
                    dobj_lemma_list[lemma_key].append([sentence,dobj_sense[0]])
            else:
                if not prep_sense:
                    sentence = predicate+ ' the <head>'+ dobj[0] +'</head> and the '+ dobj[1]+'.\n'  
                elif prep_kind == 'into':
                    sentence = predicate+ ' the <head>'+ dobj[0] +'</head> and the '+ dobj[1]+' ' +prep_kind +' ' + prepobj+'.\n' 
                else:
                    sentence = predicate+ ' the <head>'+ dobj[0] +'</head> and the '+ dobj[1]+' ' +prep_kind +' a ' + prepobj+'.\n' 

                lemma_key = dobj_lemma[0] 
                if lemma_key not in dobj_lemma_list.keys():
                    dobj_lemma_list[lemma_key] = [[sentence,dobj_sense[0]]]
                else:
                    dobj_lemma_list[lemma_key].append([sentence,dobj_sense[0]])
                
                if not prep_sense:
                    sentence = predicate+ ' the '+ dobj[0] +' and the <head>'+ dobj[1]+' </head> .\n' 
                elif prep_kind == 'into':
                    sentence = predicate+ ' the '+ dobj[0] +' and the <head>'+ dobj[1]+' </head> ' +prep_kind +' ' + prepobj+'.\n' 
                else:
                    sentence = predicate+ ' the '+ dobj[0] +' and the <head>'+ dobj[1]+' </head> ' +prep_kind +' a ' + prepobj+'.\n' 

                lemma_key = dobj_lemma[1] 
                if lemma_key not in dobj_lemma_list.keys():
                    dobj_lemma_list[lemma_key] = [[sentence,dobj_sense[1]]]
                else:
                    dobj_lemma_list[lemma_key].append([sentence,dobj_sense[1]])    
                    
        
            
    #Create xml and key file
    for x in range(0,len(predicate_lemma_list.keys())):
        lexelt = ET.SubElement(corpus, 'lexelt', item=predicate_lemma_list.keys()[x],pos="unk")
        
        for y in range(0,len(predicate_lemma_list[predicate_lemma_list.keys()[x]])):
            instance = predicate_lemma_list[predicate_lemma_list.keys()[x]][y]
            sentence = instance[0]
            sense = instance[1]
            id_str = predicate_lemma_list.keys()[x]+"."+str(y)
            key_file.write("{} {} {}\n".format(predicate_lemma_list.keys()[x],id_str,sense))
            
            instance = ET.SubElement(lexelt, 'instance', id=id_str,docscr="SEMCORE")
            #instance = ET.SubElement(lexelt, 'instance', id=id_str)
            context = ET.SubElement(instance, 'context')
            context.text = sentence
    
    for x in range(0,len(dobj_lemma_list.keys())):
        lexelt = ET.SubElement(corpus, 'lexelt', item=dobj_lemma_list.keys()[x],pos="unk")
        
        for y in range(0,len(dobj_lemma_list[dobj_lemma_list.keys()[x]])):
            instance = dobj_lemma_list[dobj_lemma_list.keys()[x]][y]
            sentence = instance[0]
            sense = instance[1]
            id_str = dobj_lemma_list.keys()[x]+"."+str(y)
            key_file.write("{} {} {}\n".format(dobj_lemma_list.keys()[x],id_str,sense))
            
            instance = ET.SubElement(lexelt, 'instance', id=id_str,docscr="SEMCORE")
            #instance = ET.SubElement(lexelt, 'instance', id=id_str)
            context = ET.SubElement(instance, 'context')
            context.text = sentence
            
    tree = ET.ElementTree(corpus)
    tree.write(os.path.join(result_path,'train.xml'))
    key_file.close()
    
    tree_file = open(os.path.join(result_path,'train.xml'),'r')
    tree_content = tree_file.read()
    tree_file.close()
    os.remove(os.path.join(result_path,'train.xml'))
    
    tree_file = open(os.path.join(result_path,'train.xml'),'w')
    tree_content = tree_content.replace("&lt;","<")   
    tree_content = tree_content.replace("&gt;",">")
    tree_file.write(tree_content)
    tree_file.close()

def create_IMS_dbs(mln,dbfile):
    dbs = []
    dbs_mln = readDBFromFile(mln,dbfile)
    dbs_as_textfiles = regex_new_db.split(open(dbfile,'r').read())
    
    for i in range(0,len(dbs_mln)):
        sentence = dbs_as_textfiles[i].split('\n')[0].replace("/","")
        dbs.append(IMSDatabase(sentence,dbs_mln[i]))
    
    return dbs
                    
def doXVal(folds, multicore, dbfile,testSetCount=1):
    
    directory = os.path.join(str(testSetCount),time.strftime("%a_%d_%b_%Y_%H:%M:%S_K="+str(folds)+"_TSC="+str(testSetCount)+"_"+str(os.path.basename(dbfile))+"_", time.localtime()))
    os.mkdir(directory)
    dbs = create_IMS_dbs(MLN,dbfile) 
        
    if len(dbs) < folds:
        print 'Cannot do %d-fold cross validation with only %d databases.' % (folds, len(dbs))
        exit(0)
    
    shuffle(dbs)
    partSize = int(math.ceil(len(dbs)/float(folds)))
    partition = []
    for i in range(folds):
        partition.append(dbs[i*partSize:(i+1)*partSize])
    
    
    foldRunnables = []
    for foldIdx in range(folds):
        partion_ = list(partition)
        params = XValFoldParams()
        params.testDBs = []
        params.learnDBs = []
        
        for i in range(0,testSetCount):
            if (foldIdx >= len(partion_)):
                params.testDBs.extend(partion_[0])
                del partion_[0]
            else:     
                params.testDBs.extend(partion_[foldIdx])
                del partion_[foldIdx]
        
        for part in partion_:
            params.learnDBs.extend(part)
            
        print 'LEARN DBS :' + str(len(params.learnDBs))
        print 'TEST DBS :' + str(len(params.testDBs))
        
        params.foldIdx = foldIdx
        params.foldCount = folds
        params.directory = directory
        #Remove folds which are not containing tests or learn due to uneven num_of_test_sets/k
        if((len(params.learnDBs) > 0) and (len(params.testDBs) > 0)):
            foldRunnables.append(XValFold(params))
    
    if multicore:
        # set up a pool of worker processes
        try:
            workerPool = Pool(maxtasksperchild=1)
            result = workerPool.map_async(runFold, foldRunnables).get()
            workerPool.close()
            workerPool.join()
            cm = ConfusionMatrix()
            
            for r in result:
                cm.combine(r)
            cm_path = os.path.join(directory,os.path.basename(directory)+'.cm')
            
            print cm_path
            cm.toFile(cm_path)
            cm_path_list.append(cm_path)
            
        except (KeyboardInterrupt, SystemExit, SystemError):
            workerPool.terminate()
            workerPool.join()
            print traceback.format_exc()
            exit(1)
            
        except:
            print traceback.format_exc()
            exit(1)
#     startTime = time.time()
    else:
        
        for fold in foldRunnables:
            runFold(fold)
mapping_noun_dict = get_wn30_to_wn171_dict('wn30-171.noun')
mapping_verb_dict = get_wn30_to_wn171_dict('wn30-171.verb')
mapping_adj_dict = get_wn30_to_wn171_dict('wn30-171.adj')
wordnet_index_sense = get_wordnet_index_sense()

if __name__ == '__main__':
    args = sys.argv[1:]
    
    if len(args) == 2:
        input_dir = args[0]
        MLN = readMLNFromFile(args[1], grammar='PRACGrammar', logic='FirstOrderLogic')
        
        file_list = []
        
        for filename in os.listdir(input_dir):
            file_list.append(os.path.join(input_dir,filename))
        for x in range(9,0,-1):
            cm_path_list = []
            os.mkdir(str(x))
            for f in file_list:
                doXVal(10, True, f,x)
                
            ConfusionMatrix.write_comparison_results_between_confusion_matrices(os.path.join(str(x),'OVERALL_RESULT_{}'.format(str(x))), *cm_path_list)
        
        
        
    