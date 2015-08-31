# 
#
# (C) 2011-2014 by Daniel Nyga (nyga@cs.uni-bremen.de)
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

from Tkinter import _setit, Tk, Frame, Label, StringVar, OptionMenu, Entry,\
    IntVar, Checkbutton, Button
import sys
import os
import time
import re
from tkMessageBox import askyesno
import traceback
import subprocess
import logging
import StringIO
from Tkconstants import BOTH, W, LEFT, NE, E

# --- inference class ---
from prac.core import PRACKnowledgeBase
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
from prac.pracutils.RolequeryHandler import RolequeryHandler
from pracmln import MLN, Database
from pracmln.mln.methods import InferenceMethods
from pracmln.mln.util import balancedParentheses, ifNone, colorize
from pracmln.praclog import logger
from pracmln.utils.config import query_config_pattern, PRACMLNConfig
from pracmln.utils.widgets import FilePickEdit
from pracmln.utils import config


def nop(*args, **kwargs): pass

class MLNInfer(object):
    def __init__(self):
        self.pymlns_methods = InferenceMethods.names()
        self.default_settings = {"numChains":"1", "maxSteps":"", "saveResults":False, "convertAlchemy":False, "openWorld":True} # a minimal set of settings required to run inference
    
    def run(self, mlnFiles, evidenceDB, method, queries, engine="PRACMLNs", output_filename=None, params="", **settings):
        '''
            runs an MLN inference method with the given parameters
        
            mlnFiles: list of one or more MLN input files
            evidenceDB: name of the MLN database file from which to read evidence data
            engine: either "PyMLNs"/"internal", "J-MLNs" or one of the keys in the configured Alchemy versions (see configMLN.py)
            method: name of the inference method
            queries: comma-separated list of queries
            output_filename (compulsory only when using Alchemy): name of the file to which to save results
                For the internal engine, specify saveResults=True as an additional settings to save the results
            params: additional parameters to pass to inference method. For the internal engine, it is a comma-separated
                list of assignments to parameters (dictionary-type string), for the others it's just a string of command-line
                options to pass on
            settings: additional settings that control the inference process, which are usually set by the GUI (see code)
                
            returns a mapping (dictionary) from ground atoms to probability values.
                For J-MLNs, results are only returned if settings are saved to a file (settings["saveResults"]=True and output_filename given)
        '''
        self.gconf = dict(self.default_settings)        
        self.gconf.update(settings)
        input_files = mlnFiles
        db = evidenceDB
        query = queries
        
        params = {}

#         results_suffix = ".results"
#         output_base_filename = output_filename
#         if output_base_filename[-len(results_suffix):] == results_suffix:
#             output_base_filename = output_base_filename[:-len(results_suffix)]
        
        # determine closed-world preds
        cwPreds = []
        if "cwPreds" in self.gconf:            
            cwPreds = filter(lambda x: x != "", map(str.strip, self.gconf["cwPreds"].split(",")))
        
        # collect inference arguments
        args = {"details":True, "shortOutput":True, "debugLevel":1}
        args.update(eval("dict(%s)" % params)) # add additional parameters
        # set the debug level
        logging.getLogger().setLevel(eval('logging.%s' % args.get('debug', 'WARNING').upper()))

        if self.gconf["numChains"] != "":
            args["numChains"] = int(self.gconf["numChains"])
        if self.gconf["maxSteps"] != "":
            args["maxSteps"] = int(self.gconf["maxSteps"])
        outFile = None
        if self.gconf["saveResults"]:
            haveOutFile = True
            outFile = file(output_filename, "w")
            args["outFile"] = outFile
        args['useMultiCPU'] = self.gconf.get('useMultiCPU', False)
        args["probabilityFittingResultFileName"] = output_base_filename + "_fitted.mln"

        print args
        # engine-specific handling
        if engine in ("internal", "PRACMLNs"): 
            try:
                print "\nStarting %s...\n" % method
                
                # read queries
                queries = []
                q = ""
                for s in map(str.strip, query.split(",")):
                    if q != "": q += ','
                    q += s
                    if balancedParentheses(q):
                        queries.append(q)
                        q = ""
                if q != "": raise Exception("Unbalanced parentheses in queries!")
                
                # create MLN
                # mln = MLN.MLN(input_files, verbose=verbose, defaultInferenceMethod=MLN.InferenceMethods.byName(method))
                mln = MLN(mlnfile=input_files, logic=self.gconf['logic'], grammar=self.gconf['grammar'])#, verbose=verbose, defaultInferenceMethod=MLN.InferenceMethods.byName(method))
                mln.defaultInferenceMethod = InferenceMethods.byName(method)
                # set closed-world predicates
                for pred in cwPreds:
                    mln.setClosedWorldPred(pred)
                
                # create ground MRF
                start = time.time()
                mrf = mln.groundMRF(db, verbose=args.get('verbose', False), method='FastConjunctionGrounding')
                groundingTime = time.time() - start
                print 'Grounding took %.2f sec.' % groundingTime

                # check for print/write requests
                if "printGroundAtoms" in args:
                    if args["printGroundAtoms"]:
                        mrf.printGroundAtoms()
                if "printGroundFormulas" in args:
                    if args["printGroundFormulas"]:
                        mrf.printGroundFormulas()
                if "writeGraphML" in args:
                    if args["writeGraphML"]:
                        graphml_filename = output_base_filename + ".graphml"
                        print "writing ground MRF as GraphML to %s..." % graphml_filename
                        mrf.writeGraphML(graphml_filename)
                # invoke inference and retrieve results
                mrf.infer(queries, **args)
                results = {}
                for gndFormula, p in mrf.getResultsDict().iteritems():
                    results[str(gndFormula)] = p
                
                # close output file and open if requested
                if outFile != None:
                    outFile.close()
            except:
                cls, e, tb = sys.exc_info()
                sys.stderr.write("Error: %s\n" % str(e))
                traceback.print_tb(tb)
                
        # open output file in editor
        if False and haveOutFile and config.query_edit_outfile_when_done: # this is mostly useless
            editor = config.editor
            params = [editor, output_filename]
            print 'starting editor: %s' % subprocess.list2cmdline(params)
            subprocess.Popen(params, shell=False)
            
        return results

# --- main gui class ---

class PRACQueryGUI(object):

    def __init__(self, pracinference, conf, directory='.'):
        log = logger(__name__)
        self.prac = pracinference.prac
        prac = self.prac
        self.prac_inference = pracinference
        self.infStep = None

        self.initialized = False
        self.directory = directory # TODO remove
        self.module_dir = os.path.join(os.environ['PRAC_HOME'], 'pracmodules', 'wnsenses')
        self.master = Tk()
        self.master.bind('<Return>', self.start)
        self.master.bind('<Escape>', lambda a: self.master.quit())
        self.master.title("PRAC Query Tool")
        master = self.master


        self.config = None
        self.gconf = conf # TODO remove

        self.frame = Frame(master)
        self.frame.pack(fill=BOTH, expand=1)
        self.frame.columnconfigure(1, weight=1)

        # module selection
        row = 0        
        Label(self.frame, text="Module: ").grid(row=row, column=0, sticky="E")
        modules = [module for module in prac.moduleManifestByName]
        self.selected_module = StringVar(master)
        self.selected_module.trace("w", self.onChangeModule)
        l = OptionMenu(self.frame, self.selected_module, *tuple(modules))
        l.grid(row=row, column=1, sticky="NWE")
        
        # Knowledge Base selection
        row += 1
        Label(self.frame, text="Knowledge Base: ").grid(row=row, column=0, sticky="E")
        self.selected_kb = StringVar(master)
        self.selected_kb.trace("w", self.onChangeKB)
        self.drop_down_kb_selection = OptionMenu(self.frame, variable=self.selected_kb, value=None)
        self.drop_down_kb_selection.grid(row=row, column=1, sticky="NEWS")
        
        # Save KB
        row += 1
        Label(self.frame, text="Save KB: ").grid(row=row, column=0, sticky="NE")
        saveKBFrame = Frame(self.frame)
        saveKBFrame.grid(row=row, column=1, sticky="NEWS")
        saveKBFrame.columnconfigure(0, weight=1)
        
        # KB name
        self.kb_name = StringVar(master)
        self.kb_name.set('%s' % self.gconf.get("selected_kb", ""))
        self.entry_kb_filename = Entry(saveKBFrame, textvariable = self.kb_name)
        self.entry_kb_filename.grid(row=0, column=0, sticky="NEWS")
        # save button
        self.save_button = Button(self.frame, text="save", command=self.onSaveKB, height=1)
        self.save_button.grid(row=row, column=1, sticky="NES")
        
        # logic selection
        row += 1
        Label(self.frame, text='Logic: ').grid(row=row, column=0, sticky='E')
        logics = ['FirstOrderLogic', 'FuzzyLogic']
        self.selected_logic = StringVar(master)
        logic = self.gconf.get('logic')
        if not logic in logics: logic = logics[0]
        self.selected_logic.set(logic)
        self.selected_logic.trace('w', self.onChangeLogic)
        l = apply(OptionMenu, (self.frame, self.selected_logic) + tuple(logics))
        l.grid(row=row, column=1, sticky='NWE')
        
        # mln selection
        row += 1
        Label(self.frame, text="MLN: ").grid(row=row, column=0, sticky=NE)
        self.selected_mln = FilePickEdit(self.frame, config.query_mln_filemask, 
                                         self.gconf.get("mln", ""), 22, self.changedMLN, 
                                         rename_on_edit=self.gconf.get("mln_rename", 0), 
                                         font=config.fixed_width_font, coloring=True,
                                         directory=os.path.join(self.module_dir,'mln'))

        self.selected_mln.grid(row=row, column=1, sticky="NWES")
        self.frame.rowconfigure(row, weight=1)

        # option: use model extension
        self.use_emln = IntVar()
        self.cb_use_emln = Checkbutton(self.selected_mln.options_frame, text="use model extension", 
                                       variable=self.use_emln)
        self.cb_use_emln.pack(side=LEFT)
        self.use_emln.set(self.gconf.get("useEMLN", 0))
        self.use_emln.trace("w", self.onChangeUseEMLN)
        # mln extension selection
        self.selected_emln = FilePickEdit(self.selected_mln, "*.emln", None, 12, None, 
                                          rename_on_edit=self.gconf.get("emln_rename", 0),
                                          font=config.fixed_width_font, coloring=True)
        self.onChangeUseEMLN()

        # evidence database selection
        row += 1
        Label(self.frame, text="Evidence: ").grid(row=row, column=0, sticky=NE)
        self.selected_db = FilePickEdit(self.frame, config.query_db_filemask, self.gconf.get("db", ""), 
                                        12, self.changedDB, rename_on_edit=self.gconf.get("db_rename", 0),
                                        font=config.fixed_width_font, coloring=True, directory=os.path.join(self.module_dir,'db'))
        self.selected_db.grid(row=row,column=1, sticky="NWES")
        self.frame.rowconfigure(row, weight=1)

        # inference method selection
        row += 1
        self.inference = MLNInfer()
        self.list_methods_row = row
        Label(self.frame, text="Method: ").grid(row=row, column=0, sticky=E)
        self.selected_method = StringVar(master)
        self.list_methods = OptionMenu(self.frame, self.selected_method, *InferenceMethods.names())
        self.list_methods.grid(row=self.list_methods_row, column=1, sticky="NWE")

        # queries
        row += 1
        Label(self.frame, text="Queries: ").grid(row=row, column=0, sticky=E)
        self.query = StringVar(master)
        self.query.set(self.gconf.get("query", "foo"))
        Entry(self.frame, textvariable = self.query).grid(row=row, column=1, sticky="NEW")

        # query formula selection
        #row += 1
        #Label(self.frame, text="Query formulas: ").grid(row=row, column=0, sticky=NE)
        self.selected_qf = FilePickEdit(self.frame, "*.qf", self.gconf.get("qf", ""), 6)
        #self.selected_qf.grid(row=row,column=1)

        #  parameters
        row += 1
        Label(self.frame, text="Parameters: ").grid(row=row, column=0, sticky="NE")
        self.params = StringVar(master)
        self.entry_params = Entry(self.frame, textvariable = self.params)
        self.entry_params.grid(row=row, column=1, sticky="NEW")

        # closed-world predicates
        row += 1
        Label(self.frame, text="CW preds: ").grid(row=row, column=0, sticky="NE")
        self.cwPreds = StringVar(master)
        self.cwPreds.set(self.gconf.get("cwPreds", ""))
        self.entry_cw = Entry(self.frame, textvariable = self.cwPreds)
        self.entry_cw.grid(row=row, column=1, sticky="NEW")

        # all preds open-world
        option_container = Frame(self.frame)
        option_container.grid(row=row, column=1, sticky="NES")
        row += 1
        self.closed_world = IntVar()
        self.cb_closed_world = Checkbutton(option_container, text="Apply CW assumption to all except queries", variable=self.closed_world)
        self.cb_closed_world.grid(row=0, column=1, sticky=W)
        self.closed_world.set(self.gconf.get("closedWorld", True))
        
        # Multiprocessing 
        self.multicore = IntVar()
        self.cb_use_multiCPU = Checkbutton(option_container, text="Use all CPUs", variable=self.multicore)
        self.cb_use_multiCPU.grid(row=0, column=2, sticky=W)
        self.multicore.set(self.gconf.get("useMultiCPU", False))

        # start and continue buttons
        self.btn_container = Frame(self.frame)
        self.btn_container.grid(row=row, column=1, sticky='EW')
        
        start_button = Button(self.btn_container, text="Start Inference", command=self.start)
        start_button.grid(row=0, column=1, sticky='E')
        
        continue_button = Button(self.btn_container, text="Continue >", command=self.onContinue)
        continue_button.grid(row=0, column=2, sticky='W')

        self.initialized = True
        kb = self.gconf.get("module", modules[0])
        self.selected_module.set(kb)
        self.setDatabases(*pracinference.inference_steps[-1].output_dbs)
#         self.onChangeModule()
#         self.onChangeKB()
        self.setGeometry()

    def setGeometry(self):
        g = self.gconf.get("geometry")
        if g is None: return
        # this is a hack: since geometry apparently does not work as expected
        # (at least under Ubuntu: the main window is not put at the same position
        # where it has been before), do this correction of coordinates.
        re_pattern = r'([\-0-9]+)x([\-0-9]+)\+([\-0-9]+)\+([\-0-9]+)'
        (w_old, h_old, x_old, y_old) = map(int, re.search(re_pattern, g).groups())
        self.master.geometry(g)
        new_g = self.master.winfo_geometry()
        (w_new, h_new, x_new, y_new) = map(int, re.search(re_pattern, new_g).groups())
        (w_diff, h_diff, x_diff, y_diff) = (w_old-w_new, h_old-h_new, x_old-x_new, y_old-y_new)
        (w_new, h_new, x_new, y_new) = (w_old, h_old, x_new-x_diff, y_new-y_diff)
        self.master.geometry('%dx%d+%d+%d' % (w_new, h_new, x_new, y_new))
         
    def onSaveKB(self):
        print 'kbname', self.kb_name.get()
        self.config = PRACMLNConfig(os.path.join(self.module_dir, 'bin', query_config_pattern % self.kb_name.get()))
        self.config["mln_rename"] = self.selected_mln.rename_on_edit.get()
        self.config["db"] = self.selected_db.get()
        self.config['mln'] = self.selected_mln.get()
        self.config["db_rename"] = self.selected_db.rename_on_edit.get()
        self.config["method"] = InferenceMethods.id(str(self.selected_method.get()))
        self.config["params"] = str(self.params.get())
        self.config["queries"] = self.query.get()
        self.config['emln'] = self.selected_emln.get().encode('utf8')
        self.config["cw"] = self.closed_world.get()
        self.config["cw_preds"] = self.cwPreds.get()
        self.config["use_emln"] = self.use_emln.get()
        self.config['logic'] = self.selected_logic.get()
        self.config['multicore'] = self.multicore.get()
        self.config.dump()


    def onChangeKB(self, name = None, index = None, mode = None):
        self.kb_name.set(self.selected_kb.get())
        if self.selected_kb.get() == 'new':
            return

        confname = os.path.join(self.module_dir, 'bin', query_config_pattern % self.selected_kb.get())
        if self.config is None or not self.initialized or os.path.exists(confname):
            self.set_config(PRACMLNConfig(confname))


    def changedMLN(self, mlnname): pass


    def changedDB(self, name):
        self.db_filename = name
#         self.setOutputFilename()
        # restore stored query (if any)
#         query = self.gconf["queryByDB"].get(name)
#         if query is None: # try file
#             query_file = "%s.query" % name
#             if os.path.exists(query_file) and "query" in dir(self):
#                 f = file(query_file, "r")
#                 query = f.read()
#                 f.close()
#         if not query is None and hasattr(self, "query"):
#             self.query.set(query)
#         # select EMLN
#         emln = self.gconf["emlnByDB"].get(name)
#         if not emln is None:
#             self.selected_emln.set(emln)
            

    def onChangeModule(self, *args):
        module_path = self.prac.moduleManifestByName[self.selected_module.get()].module_path
        self.set_module_dir(module_path)
        self.updateKBList()

    def set_module_dir(self, dirpath):
        dirpath = os.path.abspath(dirpath)
        self.selected_mln.setDirectory(os.path.join(dirpath, 'mln'))
        self.selected_emln.setDirectory(os.path.join(dirpath, 'emln'))
        self.selected_db.setDirectory(os.path.join(dirpath, 'db'))
        self.module_dir = dirpath

    def updateKBList(self):
        modulename = self.selected_module.get()
        kbs = ['new']
        module_path = self.prac.moduleManifestByName[modulename].module_path

        if os.path.exists(os.path.join(module_path, 'bin')):
            for path in os.listdir(os.path.join(module_path, 'bin')):
                if os.path.isdir(path): continue
                if path.endswith('.query.conf'):
                    kbs.append(path[0:path.rfind('.query.conf')])

        # remove all items
        menu = self.drop_down_kb_selection["menu"] 
        menu.delete(0, 'end')

        # add the new ones
        for item in kbs:
            menu.add_command(label=item, command=_setit(self.selected_kb, item, None))
        self.selected_kb.set(kbs[0])

    def onChangeUseMultiCPU(self, *args):
        pass

    def onChangeUseEMLN(self, *args):
        if self.use_emln.get() == 0:
            self.selected_emln.grid_forget()
        else:
            self.selected_emln.grid(row=self.selected_mln.row+1, column=0, sticky="NWES")

    def onChangeLogic(self, name = None, index = None, mode = None):
        pass
    
    def set_config(self, conf):
        self.config = conf
        self.selected_logic.set(ifNone(conf['logic'], 'FirstOrderLogic'))
        self.selected_mln.select(ifNone(conf['mln'], ''))
        self.selected_mln.rename_on_edit.set(ifNone(conf['mln_rename'], 0))
        self.selected_db.select(ifNone(conf['db'], ''))
        self.selected_db.rename_on_edit.set(ifNone(conf['db_rename'], False))
        self.selected_method.set(ifNone(self.config["method"], InferenceMethods.name('MCSAT'), transform=InferenceMethods.name))
        self.selected_emln.set(ifNone(conf['emln'], ''))
        self.multicore.set(ifNone(conf['multicore'], False))
        self.params.set(ifNone(conf['params'], ''))
        self.use_emln.set(ifNone(conf['use_emln'], False))
        if self.use_emln.get():
            self.selected_emln.select(self.use_emln.get())
        self.cwPreds.set(ifNone(conf['cw_preds'], ''))
        self.closed_world.set(ifNone(conf['cw'], False))
        self.query.set(ifNone(conf['queries'], 'foo, bar'))
        self.selected_emln.set(ifNone(conf['use_emln'], False))


    def getTempKBFromGUI(self):
        log = logging.getLogger(self.__class__.__name__)
        params = {}
        kb = PRACKnowledgeBase(self.prac)
        kb.set_querymln(self.selected_mln.get_text(), path=os.path.join(self.prac.moduleManifestByName[self.selected_module.get()].module_path, 'mln'))
        kb.dbs = list(Database(kb.query_mln, dbfile=str(self.selected_db.get_text())))
        params['queries'] = self.query.get()
        params['method'] = InferenceMethods.name2value.get(self.selected_method.get(), '')
        params.update(eval("dict(%s)" % self.params.get()))
        params['cwPreds'] = filter(lambda x: x != "", map(str.strip, self.cwPreds.get().split(",")))
        params['closedWorld'] = self.closed_world.get()
        params['logic'] = self.selected_logic.get()
        params['useMultiCPU'] = self.multicore.get()
        kb.query_params = params
        log.info(params)
        return kb


    def start(self, saveGeometry = True):

        log = logging.getLogger(self.__class__.__name__)
        # collect the parameters and create a temporary KB
        
        if saveGeometry:
            self.gconf["geometry"] = self.master.winfo_geometry()
        
        # hide main window
        self.master.withdraw()
        # runinference
        try:
            module = self.prac.getModuleByName(self.selected_module.get())
            oldDebugMode = logging.getLogger().level
            kb = self.getTempKBFromGUI()
            if 'debug' in kb.query_params:
                log.info('Setting Debug level to %s' % kb.query_params['debug'])
                logging.getLogger().setLevel(kb.query_params['debug'])
            self.infStep = module(self.prac_inference, kb=kb)
            logging.getLogger().setLevel(oldDebugMode)
            
        except:
            cls, e, tb = sys.exc_info()
            sys.stderr.write("Error: %s\n" % str(e))
            traceback.print_tb(tb)

        # restore main window
        self.master.deiconify()
        self.setGeometry()
        # reload the files (in case they changed)
#         self.selected_mln.reloadFile()
#         self.selected_db.reloadFile()

        sys.stdout.flush()
        
        
    def onContinue(self):
        if self.infStep is None:
            self.start()
        self.prac_inference.inference_steps.append(self.infStep)
        self.setDatabases(*self.infStep.output_dbs)
        print 'Input databases have been replaced by the latest results.'
        
        
    def setDatabases(self, *dbs):
        strBuf = StringIO.StringIO()
        for i, db in enumerate(dbs):
            db.write(strBuf, color=False)
            if i < len(dbs) - 1:
                strBuf.write('---\n')
        strBuf.seek(0)
        self.selected_db.setText(strBuf.getvalue().encode('utf-8'))
        strBuf.close()


    def open(self):
        self.master.mainloop()
        

# -- main app --



if __name__ == '__main__':

    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False, action='store_true',
                      help="Starts PRAC inference with an interactive GUI tool.")
    (options, args) = parser.parse_args()

    interactive = options.interactive

    sentences = args

    log = logging.getLogger()
    log.setLevel(logging.INFO)
    actionRoles = None
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)

    infer = PRACInference(prac, sentences)
#     actionCore.insertdbs(infer, *readDBFromString(prac.mln, dbs))

    if interactive: # use the GUI
        # in case we have natural-language parameters, parse them
        if len(infer.instructions) > 0:
            parser = prac.getModuleByName('nl_parsing')
            prac.run(infer, parser)
        conf = PRACMLNConfig()
        gui = PRACQueryGUI(infer, conf)
        gui.open()
    else: # regular PRAC pipeline
        while infer.next_module() != None :
            modulename = infer.next_module()
            module = prac.getModuleByName(modulename)
            prac.run(infer,module)
#             if modulename == 'senses_and_roles':
#                 module.role_distributions(infer)
#                 exit(0)


    step = infer.inference_steps[-1]
    print
    print colorize('+========================+',  (None, 'green', True), True)
    print colorize('| PRAC INFERENCE RESULTS |',  (None, 'green', True), True)
    print colorize('+========================+',  (None, 'green', True), True)
    wordnet_module = prac.getModuleByName('wn_senses')


#             db.write(sys.stdout, color=True)
    for db in step.output_dbs:
        for a in sorted(db.evidence.keys()):
            v = db.evidence[a]
            if v > 0.001 and (a.startswith('action_core') or a.startswith('has_sense') or a.startswith('achieved_by')):
                if a.startswith('has_sense'):

                    group = re.split(',',re.split('has_sense\w*\(|\)',a)[1])
                    word = group[0];
                    sense = group[1];
                    if sense != 'null':
                        print
                        print colorize('  WORD:', (None, 'white', True), True), word,
                        print colorize('  SENSE:', (None, 'white', True), True), sense
                        wordnet_module.printWordSenses(wordnet_module.get_possible_meanings_of_word(db, word), sense)
                        print
                else:
                    print '%.3f    %s' % (v, a)
        RolequeryHandler.queryRolesBasedOnActioncore(db).write(color=True)

    if hasattr(step, 'executable_plans'):
        print
        print colorize('+==========================+',  (None, 'green', True), True)
        print colorize('| PARAMETERIZED ROBOT PLAN |',  (None, 'green', True), True)
        print colorize('+==========================+',  (None, 'green', True), True)
        print
        for plan in step.executable_plans:
            print plan


