from concurrent.futures import ProcessPoolExecutor as Pool
from multiprocessing import cpu_count

from prac.db.ies.ies_utils import MultiprocessingMethods

from prac.db.ies.ies_controller.FrameExtractor import FrameExtractor
from prac.db.ies.ies_models.FrameExtractorResult import FrameExtractorResult

def run_frame_extraction_process(howtos):
    frame_extractor = FrameExtractor(howtos)
    return frame_extractor.extract_frames()

def analyze_howto(howtos,use_multicore=False):
    
    if use_multicore:
        result = FrameExtractorResult()
        
        if len(howtos) >= cpu_count(): 
            corpus_sub_lists = MultiprocessingMethods.split_list_in_n_equal_sub_lists(howtos, cpu_count())
        else:
            corpus_sub_lists = [[howto] for howto in howtos]
        
        workerPool = Pool(max_workers=cpu_count())
        temp_result_list = list(workerPool.map(run_frame_extraction_process, corpus_sub_lists))
        workerPool.shutdown()
        
        for temp_result in temp_result_list:
            result.merge_frame_extractor_result(temp_result)
        
        return result
    else:
        return run_frame_extraction_process(howtos)


