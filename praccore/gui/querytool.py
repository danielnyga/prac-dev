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
import struct
import time
import re
from fnmatch import fnmatch
import traceback
import widgets
# from widgets import *
import configMLN as config
import mln
import tkMessageBox
import subprocess
import shlex
from mln.util import balancedParentheses
from mln import readMLNFromFile
from mln.methods import InferenceMethods
from widgets import FilePickEdit
from logic.grammar import StandardGrammar, PRACGrammar  # @UnusedImport
import logging
from prac.core import prac_module_path, PRAC, PRACKnowledgeBase
import StringIO
from Tkconstants import BOTH, W, LEFT, NE, E
from mln.database import readDBFromString
from prac.inference import PRACInference

# --- inference class ---

def nop(*args, **kwargs): pass


class MLNInfer(object):
    def __init__(self):
        self.pymlns_methods = InferenceMethods.getNames()
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
        self.settings = dict(self.default_settings)        
        self.settings.update(settings)
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
        if "cwPreds" in self.settings:            
            cwPreds = filter(lambda x: x != "", map(str.strip, self.settings["cwPreds"].split(",")))
        
        # collect inference arguments
        args = {"details":True, "shortOutput":True, "debugLevel":1}
        args.update(eval("dict(%s)" % params)) # add additional parameters
        # set the debug level
        logging.getLogger().setLevel(eval('logging.%s' % args.get('debug', 'WARNING').upper()))

        if self.settings["numChains"] != "":
            args["numChains"] = int(self.settings["numChains"])
        if self.settings["maxSteps"] != "":
            args["maxSteps"] = int(self.settings["maxSteps"])
        outFile = None
        if self.settings["saveResults"]:
            haveOutFile = True
            outFile = file(output_filename, "w")
            args["outFile"] = outFile
        args['useMultiCPU'] = self.settings.get('useMultiCPU', False)
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
                mln = readMLNFromFile(input_files, logic=self.settings['logic'], grammar=self.settings['grammar'])#, verbose=verbose, defaultInferenceMethod=MLN.InferenceMethods.byName(method))
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

    def __init__(self, pracinference, directory='.'):
        self.prac = pracinference.prac
        prac = self.prac
        self.prac_inference = pracinference
        self.inference_step = None

        self.initialized = False
        self.directory = directory
        self.master = Tk()
        self.master.title("PRAC Query Tool")
        master = self.master
        
        
        self.settings = {}
#         if not "queryByDB" in self.settings: self.settings["queryByDB"] = {}
#         if not "emlnByDB" in self.settings: self.settings["emlnByDB"] = {}
#         if not "use_multiCPU" in self.settings: self.settings['use_multiCPU'] = False 
        
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
        self.kb_name.set('%s' % self.settings.get("selected_kb", ""))
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
        logic = self.settings.get('logic')
        if not logic in logics: logic = logics[0]
        self.selected_logic.set(logic)
        self.selected_logic.trace('w', self.onChangeLogic)
        l = apply(OptionMenu, (self.frame, self.selected_logic) + tuple(logics))
        l.grid(row=row, column=1, sticky='NWE')
        
        # mln selection
        row += 1
        Label(self.frame, text="MLN: ").grid(row=row, column=0, sticky=NE)
        self.selected_mln = FilePickEdit(self.frame, config.query_mln_filemask, 
                                         self.settings.get("mln", ""), 22, self.changedMLN, 
                                         rename_on_edit=self.settings.get("mln_rename", 0), 
                                         font=config.fixed_width_font, coloring=config.coloring,
                                         directory='.')
        self.selected_mln.grid(row=row, column=1, sticky="NWES")
        self.frame.rowconfigure(row, weight=1)

        # option: use model extension
        self.use_emln = IntVar()
        self.cb_use_emln = Checkbutton(self.selected_mln.options_frame, text="use model extension", 
                                       variable=self.use_emln)
        self.cb_use_emln.pack(side=LEFT)
        self.use_emln.set(self.settings.get("useEMLN", 0))
        self.use_emln.trace("w", self.onChangeUseEMLN)
        # mln extension selection
        self.selected_emln = FilePickEdit(self.selected_mln, "*.emln", None, 12, None, 
                                          rename_on_edit=self.settings.get("mln_rename", 0), 
                                          font=config.fixed_width_font, coloring=config.coloring)
        self.onChangeUseEMLN()

        # evidence database selection
        row += 1
        Label(self.frame, text="Evidence: ").grid(row=row, column=0, sticky=NE)
        self.selected_db = FilePickEdit(self.frame, config.query_db_filemask, self.settings.get("db", ""), 
                                        12, self.changedDB, rename_on_edit=self.settings.get("emln_rename", 0), 
                                        font=config.fixed_width_font, coloring=config.coloring, directory=self.directory)
        self.selected_db.grid(row=row,column=1, sticky="NWES")
        self.frame.rowconfigure(row, weight=1)

        # inference method selection
        row += 1
        self.inference = MLNInfer()
        self.list_methods_row = row
        Label(self.frame, text="Method: ").grid(row=row, column=0, sticky=E)
        self.selected_method = StringVar(master)
        self.list_methods = OptionMenu(self.frame, self.selected_method, *InferenceMethods.getNames())
        self.list_methods.grid(row=self.list_methods_row, column=1, sticky="NWE")

        # queries
        row += 1
        Label(self.frame, text="Queries: ").grid(row=row, column=0, sticky=E)
        self.query = StringVar(master)
        self.query.set(self.settings.get("query", "foo"))
        Entry(self.frame, textvariable = self.query).grid(row=row, column=1, sticky="NEW")

        # query formula selection
        #row += 1
        #Label(self.frame, text="Query formulas: ").grid(row=row, column=0, sticky=NE)
        self.selected_qf = FilePickEdit(self.frame, "*.qf", self.settings.get("qf", ""), 6)
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
        self.cwPreds.set(self.settings.get("cwPreds", ""))
        self.entry_cw = Entry(self.frame, textvariable = self.cwPreds)
        self.entry_cw.grid(row=row, column=1, sticky="NEW")

        # all preds open-world
        option_container = Frame(self.frame)
        option_container.grid(row=row, column=1, sticky="NES")
        row += 1
        self.closed_world = IntVar()
        self.cb_closed_world = Checkbutton(option_container, text="Apply CW assumption to all except queries", variable=self.closed_world)
        self.cb_closed_world.grid(row=0, column=1, sticky=W)
        self.closed_world.set(self.settings.get("closedWorld", True))
        
        # Multiprocessing 
        self.use_multiCPU = IntVar()
        self.cb_use_multiCPU = Checkbutton(option_container, text="Use all CPUs", variable=self.use_multiCPU)
        self.cb_use_multiCPU.grid(row=0, column=2, sticky=W)
        self.use_multiCPU.set(self.settings.get("useMultiCPU", False))

        # start and continue buttons
        self.btn_container = Frame(self.frame)
        self.btn_container.grid(row=row, column=1, sticky='EW')
        
        start_button = Button(self.btn_container, text="Start Inference", command=self.start)
        start_button.grid(row=0, column=1, sticky='E')
        
        continue_button = Button(self.btn_container, text="Continue >", command=self.start)
        continue_button.grid(row=0, column=2, sticky='W')

        self.initialized = True
        kb = self.settings.get("module", modules[0])
        self.selected_module.set(kb)
        self.setDatabases(*pracinference.inference_steps[-1].output_dbs)
#         self.onChangeModule()
#         self.onChangeKB()
        self.setGeometry()

    def setGeometry(self):
        g = self.settings.get("geometry")
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
        kb = self.getTempKBFromGUI()
        kb_name = self.entry_kb_filename.get()
        module = self.prac.getModuleByName(self.selected_module.get())
        logging.getLogger(self.__class__.__name__).info('Saving KB of type %s in %s' % (kb.__class__.__name__, kb_name))
        module.save_pracmt(kb, kb_name)
        self.updateKBList()
        self.selected_kb.set(kb_name)
        

    def onChangeKB(self, name = None, index = None, mode = None):
        self.kb_name.set(self.selected_kb.get())
        if self.selected_kb.get() == 'new':
            return
        # unpickle the kb
        module = self.prac.getModuleByName(self.selected_module.get())
        kb = module.load_pracmt(self.selected_kb.get())
        self.setGUIFromKB(kb)
        
        
    def changedMLN(self, name): pass
#         self.mln_filename = name
#         self.setOutputFilename()

    def changedDB(self, name):
        self.db_filename = name
#         self.setOutputFilename()
        # restore stored query (if any)
#         query = self.settings["queryByDB"].get(name)
#         if query is None: # try file
#             query_file = "%s.query" % name
#             if os.path.exists(query_file) and "query" in dir(self):
#                 f = file(query_file, "r")
#                 query = f.read()
#                 f.close()
#         if not query is None and hasattr(self, "query"):
#             self.query.set(query)
#         # select EMLN
#         emln = self.settings["emlnByDB"].get(name)
#         if not emln is None:
#             self.selected_emln.set(emln)
            

    def onChangeModule(self, *args):
        self.updateKBList()
        
    def updateKBList(self):
        modulename = self.selected_module.get()
        kbs = ['new']
        module_path = self.prac.moduleManifestByName[modulename].module_path
        for path in os.listdir(os.path.join(module_path, 'bin')):
            if os.path.isdir(path): continue
            if path.endswith('.pracmln'):
                kbs.append(path[0:path.rfind('.pracmln')])
        kb = kbs[0]
        # remove all items
        menu = self.drop_down_kb_selection["menu"] 
        menu.delete(0, 'end')
        # add the new ones
        for item in kbs:
            menu.add_command(label=item, command=_setit(self.selected_kb, item, None))
        self.selected_kb.set(kb)
        self.selected_mln.setDirectory(os.path.join(module_path, 'mln'))
            
    def onChangeUseMultiCPU(self, *args):
        pass

    def onChangeUseEMLN(self, *args):
        if self.use_emln.get() == 0:
            self.selected_emln.grid_forget()
        else:
            self.selected_emln.grid(row=self.selected_mln.row+1, column=0, sticky="NWES")

    def onChangeLogic(self, name = None, index = None, mode = None):
        pass
    

    def getTempKBFromGUI(self):
        log = logging.getLogger(self.__class__.__name__)
        params = {}
        kb = PRACKnowledgeBase(self.prac)
        kb.set_querymln(self.selected_mln.get_text(), path=os.path.join(self.prac.moduleManifestByName[self.selected_module.get()].module_path, 'mln'))
        kb.dbs = list(readDBFromString(kb.query_mln, str(self.selected_db.get_text())))
        params['queries'] = self.query.get()
        params['method'] = InferenceMethods.name2value.get(self.selected_method.get(), '')
        params.update(eval("dict(%s)" % self.params.get()))
        params['cwPreds'] = filter(lambda x: x != "", map(str.strip, self.cwPreds.get().split(",")))
        params['closedWorld'] = self.closed_world.get()
        params['logic'] = self.selected_logic.get()
        params['useMultiCPU'] = self.use_multiCPU.get()
        kb.query_params = params
        log.info(params)
        return kb
    
    
    def setGUIFromKB(self, kb):
        self.selected_mln.setText(kb.query_mln_str)
        print dir(kb)
        p = dict(kb.query_params)
        self.query.set(p.get('queries', ''))
        if 'queries' in p: del p['queries']
        self.selected_method.set(InferenceMethods.value2name[(p.get('method', 'MCSAT'))])
        if 'method' in p: del p['method']
        self.cwPreds.set(', '.join(p.get('cwPreds', [])))
        if 'cwPreds' in p: del p['cwPreds']
        self.closed_world.set(p.get('closedWorld', False))
        if 'closedWorld' in p: del p['closedWorld']
        self.selected_logic.set(p.get('logic', 'FuzzyLogic'))
        if 'logic' in p: del p['logic']
        self.use_multiCPU.set(p.get('useMultiCPU', False))
        if 'useMultiCPU' in p: del p['useMultiCPU']
        self.params.set(','.join(map(lambda (x, y): '%s=%s' % (str(x), repr(y)), p.iteritems())))


    def start(self, saveGeometry = True):

        log = logging.getLogger(self.__class__.__name__)
        # collect the parameters and create a temporary KB
        
        if saveGeometry:
            self.settings["geometry"] = self.master.winfo_geometry()
        
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
        if self.inference_step is None:
            self.start()
        self.pracinference.inference_steps.append(self.inference_step)
        self.setDatabases(*readDBFromString(self.prac.mln, self.selected_db.get_text()))
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
    # read previously saved settings
    settings = {}
    
    prac = PRAC()
    app = PRACQueryGUI(os.path.join(prac.getManifestByName('nl_parsing').module_path, 'bin'), settings)
    

