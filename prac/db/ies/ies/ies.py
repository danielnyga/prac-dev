'''
Created on Sep 2, 2015

@author: Sebastian Koralewski (seba@informatik.uni-bremen.de)
'''
from concurrent.futures import ProcessPoolExecutor as Pool
from multiprocessing import cpu_count

import sys
from prac.db.ies.ies_utils import MultiprocessingMethods

from prac.db.ies.ies_controller.FrameExtractor import FrameExtractor
from os import path,listdir
from prac.db.ies.ies_models.FrameExtractorResult import FrameExtractorResult

def run_frame_extraction_process(corpus):
    frame_extractor = FrameExtractor(corpus)
    return frame_extractor.extract_frames()

def extract_frames_of_corpus(corpus,use_multicore=False):
    corpus_ = []
    
    if path.isdir(corpus):
        for filename in listdir(corpus):
            corpus_.append(path.abspath(path.join(corpus,filename)))
    
    elif path.isfile(corpus):
        corpus_.append(path.abspath(corpus))
    
    else:
        corpus_ = corpus
    
    if use_multicore:
        result = FrameExtractorResult()
        corpus_sub_lists = MultiprocessingMethods.split_list_in_n_equal_sub_lists(corpus_, cpu_count())
        workerPool = Pool(max_workers=cpu_count())
        temp_result_list = list(workerPool.map(run_frame_extraction_process, corpus_sub_lists))
        workerPool.shutdown()
        #workerPool.join()
        
        for temp_result in temp_result_list:
            result.merge_frame_extractor_result(temp_result)
        
        return result
    else:
        return run_frame_extraction_process(corpus_)


if __name__ == '__main__':
    args = sys.argv[1:]
    if args:
        result = extract_frames_of_corpus(args[0],True)
    else:
        print "Please provide a path to a corpus." 
        
    print 'FINISHED'
