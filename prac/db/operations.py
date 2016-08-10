from concurrent.futures import ProcessPoolExecutor as Pool
from multiprocessing import cpu_count

from prac.db.ies.ies_utils import MultiprocessingMethods

from prac.db.ies.ies_controller.FrameExtractor import FrameExtractor

def run_frame_extraction_process(howtos):
    frame_extractor = FrameExtractor(howtos)
    
    frame_extractor.extract_frames()

def analyze_howto(howtos,use_multicore=False):
    
    if use_multicore:
        if len(howtos) >= cpu_count(): 
            corpus_sub_lists = MultiprocessingMethods.split_list_in_n_equal_sub_lists(howtos, cpu_count())
        else:
            corpus_sub_lists = [[howto] for howto in howtos]
        
        workerPool = Pool(max_workers=cpu_count())
        workerPool.map(run_frame_extraction_process, corpus_sub_lists)
        workerPool.shutdown()
    
    else:
        run_frame_extraction_process(howtos)


