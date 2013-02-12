__all__ = ['Conversion']
import logging
import Conversion
import preprocessing
import re
class ConversionManager:
    """Handles conversion from different file formats into the annotationTool format.
    
    Public methods:
    __init__ --- public constructor.
    
    Public fields:
    
    """
    
#    def __init__(self):
 #       """Public constructor.
        
  #      Key arguments:
        
   #     """
        
    def _mTurkToAnno(self, x):
        return Conversion.MturkToAnno(x)
    
    def _plainToAnno(self, x):
        raise NotImplementedError

    def _ehowToAnno(self, x):
        return Conversion.EhowToAnno(x)
    
    def _jsonToAnno(self, x):
        return Conversion.JsonToAnno(x)
    
    def _textToAnno(self, x):
        return Conversion.TextToAnno(x)
    
    def _loadFile(self,filename):
        f = open(filename, 'r')
        result = []
        for l in f:
            result.append(l)
        f.close()
        return result

    def convertFiles(self, inputFiles):
        options = {'mturk' : lambda x : self._mTurkToAnno(x),
                   'plain': lambda x : self._plainToAnno(x),
                   'ehow' : lambda x : self._ehowToAnno(x),
                   'json' : lambda x : self._jsonToAnno(x),
                   ''   : lambda x: self._textToAnno(x),
        }
        result = []
        
        for f in inputFiles:
            inputString = None
            f = str(f)
            #if there is no extension
            if re.search(r"\.+$", f):
                extension = ''
                inputString = f
            else:
                extension = f.rsplit('.',1)[1]
                if extension == 'mturk':
                    logging.info('Detected Mechanical Turk File -> starting pre-processing!')
                    ic = preprocessing.indexCorrecter(f)
                    ic.saveDataToFiles()
                    filename = f.rsplit('/',1)
                    f = filename[0] + "/processed_" + filename[1]
                    logging.info('Finished pre-processing.')
            if inputString is None:
                inputString = self._loadFile(f)
                
            try:
                if not len(inputString)==0:
                    result.append(options[extension](inputString))
            except KeyError:
                logging.warning('Unknown File format for conversion.')
                return None
            except Exception:
                logging.error('Unknown error!')
                return None
        return result
        
if __name__ == "__main__":
    """Execute the file conversion with file as input parameter."""
    import sys
    
    filepath = sys.argv[1]
    cm = ConversionManager()
    print cm.convertFiles(filepath)