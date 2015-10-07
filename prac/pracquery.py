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

from Tkinter import _setit, Tk, Frame, Label, StringVar, OptionMenu, Entry,\
    IntVar, Checkbutton, Button
import sys
import os
import re
import traceback
import StringIO
from Tkconstants import BOTH, W, LEFT, NE, E

from prac.core import PRACKnowledgeBase
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
from prac.pracutils.RolequeryHandler import RolequeryHandler
from pracmln.mln.database import parse_db
from pracmln.mln.methods import InferenceMethods
from pracmln.mln.util import ifNone, colorize, out
from pracmln.praclog import logger
from pracmln.utils.config import query_config_pattern, PRACMLNConfig
from pracmln.utils.widgets import FilePickEdit
from pracmln.utils import config

log = logger(__name__)


def nop(*args, **kwargs): pass


class PRACQueryGUI(object):

    def __init__(self, master, pracinference, conf, directory='.'):
        self.prac = pracinference.prac
        prac = self.prac
        self.prac_inference = pracinference
        self.infStep = None

        self.initialized = False
        self.directory = directory # TODO remove
        self.module_dir = os.path.join(os.environ['PRAC_HOME'], 'pracmodules', 'wnsenses')
        self.master = master
        self.master.bind('<Return>', self.start)
        self.master.bind('<Escape>', lambda a: self.master.quit())
        self.master.title("PRAC Query Tool")

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
        cw_container = Frame(self.frame)
        cw_container.grid(row=row, column=1, sticky="NES")
        self.closed_world = IntVar()
        self.cb_closed_world = Checkbutton(cw_container, text="Apply CW assumption to all except queries", variable=self.closed_world)
        self.cb_closed_world.grid(row=row, column=2, sticky='E')
        self.closed_world.set(self.gconf.get("closedWorld", True))

        # Multiprocessing and verbose
        row += 1
        options_container = Frame(self.frame)
        options_container.grid(row=row, column=1, sticky='NEWS')

        self.multicore = IntVar()
        self.cb_use_multiCPU = Checkbutton(options_container, text="Use all CPUs", variable=self.multicore)
        self.cb_use_multiCPU.grid(row=0, column=0, sticky=W)
        self.multicore.set(self.gconf.get("useMultiCPU", False))
        self.verbose = IntVar()
        self.cb_use_verbose = Checkbutton(options_container, text="verbose", variable=self.verbose)
        self.cb_use_verbose.grid(row=0, column=1, sticky=W)
        self.verbose.set(self.gconf.get("verbose", False))
        self.keep_evidence = IntVar()
        self.cb_keep_evidence = Checkbutton(options_container, text="keep result", variable=self.keep_evidence)
        self.cb_keep_evidence.grid(row=0, column=2, sticky=W)
        self.keep_evidence.set(True)

        # start and continue buttons
        row += 1
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


    def onSaveKB(self):
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
        self.config['verbose'] = self.verbose.get()
        self.config.dump()


    def onChangeKB(self, *args):
        kbname = self.selected_kb.get()
        self.kb_name.set(kbname)
        if kbname == 'new':
            return

        confname = os.path.join(self.module_dir, 'bin', query_config_pattern % kbname)
        if self.config is None or not self.initialized or os.path.exists(confname):
            self.set_config(PRACMLNConfig(confname))


    def changedMLN(self, filename): pass


    def changedDB(self, filename): pass


    def onChangeModule(self, *args):
        module_path = self.prac.moduleManifestByName[self.selected_module.get()].module_path
        self.set_module_dir(module_path)
        self.updateKBList()


    def set_module_dir(self, dirpath):
        dirpath = os.path.abspath(dirpath)
        self.selected_mln.setDirectory(os.path.join(dirpath, 'mln'))
        self.selected_emln.setDirectory(os.path.join(dirpath, 'emln'))
        self.selected_db.setDirectory(os.path.join(dirpath, 'db'), keep=self.keep_evidence.get())
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


    def onChangeUseMultiCPU(self, *args): pass


    def onChangeUseEMLN(self, *args):
        if self.use_emln.get() == 0:
            self.selected_emln.grid_forget()
        else:
            self.selected_emln.grid(row=self.selected_mln.row+1, column=0, sticky="NWES")


    def onChangeLogic(self, name = None, index = None, mode = None): pass


    def set_config(self, conf):
        self.config = conf
        self.selected_logic.set(ifNone(conf['logic'], 'FirstOrderLogic'))
        self.selected_mln.select(ifNone(conf['mln'], ''))
        self.selected_mln.rename_on_edit.set(ifNone(conf['mln_rename'], 0))
        if not self.keep_evidence:
            self.selected_db.select(ifNone(conf['db'], ''))
            self.selected_db.rename_on_edit.set(ifNone(conf['db_rename'], False))
        self.selected_method.set(ifNone(self.config["method"], InferenceMethods.name('MCSAT'), transform=InferenceMethods.name))
        self.selected_emln.set(ifNone(conf['emln'], ''))
        self.multicore.set(ifNone(conf['multicore'], False))
        self.verbose.set(ifNone(conf['verbose'], False))
        self.params.set(ifNone(conf['params'], ''))
        self.use_emln.set(ifNone(conf['use_emln'], False))
        if self.use_emln.get():
            self.selected_emln.select(self.use_emln.get())
        self.cwPreds.set(ifNone(conf['cw_preds'], ''))
        self.closed_world.set(ifNone(conf['cw'], False))
        self.query.set(ifNone(conf['queries'], 'foo, bar'))
        self.selected_emln.set(ifNone(conf['use_emln'], False))


    def getTempKBFromGUI(self):
        config = PRACMLNConfig(os.path.join('/tmp', query_config_pattern % 'tmp'))
        config.update({'queries': self.query.get(),
                       'method': InferenceMethods.id(self.selected_method.get()),
                       'cw_preds': self.cwPreds.get(),
                       'cw': self.closed_world.get(),
                       'logic': self.selected_logic.get(),
                       'mln': self.selected_mln.get_filename(),
                       'emln': self.selected_emln.get_filename(),
                       'db': self.selected_db.get_filename(),
                       'multicore': self.multicore.get(),
                       'use_emln': self.use_emln.get(),
                       'verbose': self.verbose.get()
                       })
        config.update(eval("dict(%s)" % self.params.get()))
        mlncontent = str(self.selected_mln.get_text().strip())
        dbcontent = str(self.selected_db.get_text().strip())

        kb = PRACKnowledgeBase(self.prac, config)
        kb.set_querymln(mln_str=mlncontent, path=os.path.join(self.prac.moduleManifestByName[self.selected_module.get()].module_path, 'mln'))
        kb.dbs = parse_db(self.prac.mln, dbcontent, ignore_unknown_preds=True)

        return kb


    def start(self, savegeometry=True):
        if savegeometry:
            self.gconf["geometry"] = self.master.winfo_geometry()
        
        # hide main window
        self.master.withdraw()

        # runinference
        try:
            module = self.prac.getModuleByName(self.selected_module.get())
            kb = self.getTempKBFromGUI()

            self.infStep = module(self.prac_inference, kb=kb)

        except:
            cls, e, tb = sys.exc_info()
            sys.stderr.write("Error: %s\n" % str(e))
            traceback.print_tb(tb)

        # restore main window
        self.master.deiconify()


    def onContinue(self):
        if self.infStep is None:
            self.start()
        self.prac_inference.inference_steps.append(self.infStep)
        self.setDatabases(*self.infStep.output_dbs)
        log.info('Input databases have been replaced by the latest results.')
        
        
    def setDatabases(self, *dbs):
        strbuf = StringIO.StringIO()
        for i, db in enumerate(dbs):
            db.write(strbuf, bars=False, color=False)
            if i < len(dbs) - 1:
                strbuf.write('---\n')
        strbuf.seek(0)
        self.selected_db.setText(strbuf.getvalue().encode('utf-8'))
        strbuf.close()


    def open(self):
        self.master.mainloop()
        

if __name__ == '__main__':

    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False,
                                             action='store_true',
                                             help="Starts PRAC inference with an interactive GUI tool.")
    parser.add_option("-m", "--mln", dest="mln", help="the MLN model file to use")
    parser.add_option("-q", "--queries", dest="query", help="queries (comma-separated)")
    parser.add_option("-e", "--evidence", dest="db", help="the evidence database file")
    parser.add_option("-r", "--results-file", dest="output_filename", help="the results file to save")
    (options, args) = parser.parse_args()

    sentences = args
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)

    root = Tk()
    inference = PRACInference(prac, sentences)
    conf = PRACMLNConfig()

    if options.interactive: # use the GUI
        # in case we have natural-language parameters, parse them
        if len(inference.instructions) > 0:
            parser = prac.getModuleByName('nl_parsing')
            prac.run(inference, parser)
        app = PRACQueryGUI(root, inference, conf, directory=args[0] if args else None)
        app.open()
    else: # regular PRAC pipeline
        while inference.next_module() != None :
            modulename = inference.next_module()
            module = prac.getModuleByName(modulename)
            prac.run(inference, module)

        print
        print colorize('+========================+',  (None, 'green', True), True)
        print colorize('| PRAC INFERENCE RESULTS |',  (None, 'green', True), True)
        print colorize('+========================+',  (None, 'green', True), True)

        step = inference.inference_steps[-1]
        wordnet_module = prac.getModuleByName('wn_senses')
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

    if hasattr(inference.inference_steps[-1], 'executable_plans'):
        print
        print colorize('+==========================+',  (None, 'green', True), True)
        print colorize('| PARAMETERIZED ROBOT PLAN |',  (None, 'green', True), True)
        print colorize('+==========================+',  (None, 'green', True), True)
        print
        for plan in step.executable_plans:
            print plan


