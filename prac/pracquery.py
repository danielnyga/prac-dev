# (C) 2015 by Mareike Picklum (mareikep@cs.uni-bremen.de)
# 
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
# CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


from prac import locations
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
from prac.pracutils.utils import prac_heading
from pracmln import praclog
from pracmln.mln.database import parse_db
from pracmln.mln.methods import InferenceMethods
from pracmln.mln.util import ifNone, colorize, out, headline
from pracmln.utils.config import global_config_filename
from pracmln.utils.project import MLNProject, PRACMLNConfig
from pracmln.utils.widgets import FileEditBar
from prac.gui import PRACQueryGUI, DEFAULT_CONFIG
from Tkinter import Tk


logger = praclog.logger(__name__)

try:
    from pymongo import MongoClient
except ImportError:
    print logger.warning('MongoDB modules cannot be used.')



def are_requirements_set_to_load_module(module_name):
    if module_name == 'role_look_up' or module_name == 'complex_achieved_by':
        if 'pymongo' in sys.modules:
            client = MongoClient()
            try:
                database_name_list = client.database_names()
                
                if 'prac' in database_name_list:
                    database = client.prac
                    collections = database.collection_names()
                    
                    if module_name == 'role_look_up':
                        if 'howtos' in collections:
                            return True
                        else:
                            print '"Role look up" module needs a "Frames" collection.'
                            return False
                    elif module_name == 'complex_achieved_by':
                        if 'howtos' in collections:
                            return True
                        else:
                            print '"Complex achieved by module" needs a "Instructions" collection.'
                            return False
                    
                else:
                    print 'No PRAC database is stored at local MongoDB server instance.'
                    return False

            except:
                print 'No local MongoDB server instance is running.'
                return False
            #IsCollection available
        else:
            return False 
        
    return True

if __name__ == '__main__':

    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False,
                      action='store_true', help="Starts PRAC inference with an interactive GUI tool.")
    parser.add_option("-v", "--verbose", dest="verbose", default=1, type='int',
                      action="store", help="Set verbosity level {0..3}. Default is 1.")
    (options, args) = parser.parse_args()

    sentences = args
    prac = PRAC()
    prac.verbose = options.verbose

    inference = PRACInference(prac, sentences)
    conf = PRACMLNConfig(DEFAULT_CONFIG)

    if options.interactive:  # use the GUI
        root = Tk()
        # in case we have natural-language parameters, parse them
        if len(inference.instructions) > 0:
            parser = prac.module('nl_parsing')
            prac.run(inference, parser)
            #Started control structure handling
            '''
            cs_recognition = prac.module('cs_recognition')
            prac.run(inference, cs_recognition)
            
            
            dbs = inference.inference_steps[-1].output_dbs
            dbs_ = []
            
            for db in dbs:
                dbs_.extend(parser.extract_multiple_action_cores(db)) 
            inference.inference_steps[-1].output_dbs = dbs_
            '''
        app = PRACQueryGUI(root, inference, conf, directory=args[0] if args else None)
        root.mainloop()
        exit(0)
    # regular PRAC pipeline
    
    infer = PRACInference(prac, sentences)
    infer.run()
    
    print headline('inference results')
    print 'instructions:'
    for i in infer.root:
        print i
    frames = []
    for step in infer.steps():
        print step.frame
    print prac_heading('cram plans', color='blue')
    for step in infer.steps():
        if hasattr(step, 'plan'):
            print step.plan
#     infer.write()
    exit(0)
    is_inference_process_aborted = False
    
    while inference.next_module() != None and not is_inference_process_aborted:
        modulename = inference.next_module()
        
        if are_requirements_set_to_load_module(modulename):
            module = prac.module(modulename)
            prac.run(inference, module)
        else:
            if prac.verbose > 0:
                print 'Cannot infer executable plan.'
                print 'Aborting process pipeline ...'
            is_inference_process_aborted = True
    
    if not is_inference_process_aborted:
        if prac.verbose > 0:
            print prac_heading('PRAC Inference Results')

            step = inference.inference_steps[-1]
            wordnet_module = prac.module('wn_senses')
            for db in step.output_dbs:
                for a in sorted(db.evidence.keys()):
                    v = db.evidence[a]
                    if v > 0.001 and (a.startswith('action_core') or a.startswith('has_sense') or a.startswith('achieved_by')):
                        if a.startswith('has_sense'):

                            group = re.split(',', re.split('has_sense\w*\(|\)', a)[1])
                            word = group[0];
                            sense = group[1];
                            if sense != 'null':
                                if prac.verbose > 1:
                                    print
                                    print colorize('  WORD:', (None, 'white', True), True), word,
                                    print colorize('  SENSE:', (None, 'white', True), True), sense
                                    wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(db, word), sense)
                                    print
                        else:
                            if prac.verbose > 1:
                                print '%.3f    %s' % (v, a)
                for _, actioncore in db.actioncores():
                    for role, sense in db.roles(actioncore):
                        print '{}({},{})'.format(colorize(role, (None, 'white', True), True), sense, actioncore)

    if hasattr(inference.inference_steps[-1], 'executable_plans'):
        if prac.verbose > 0:
            print prac_heading('Parameterized Robot Plan')
            for plan in step.executable_plans:
                print plan
                print
