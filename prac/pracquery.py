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

from Tkinter import _setit
from Tkinter import *
import sys
import os
import re
import ntpath
from tkFileDialog import asksaveasfilename
import tkMessageBox
import traceback
import StringIO
from Tkconstants import BOTH, W, E
from prac.core.base import PRAC
from prac.core.inference import PRACInference
from prac.core.wordnet import WordNet
from prac.pracutils.RolequeryHandler import RolequeryHandler
from pracmln import praclog
from pracmln.mln.database import parse_db
from pracmln.mln.methods import InferenceMethods
from pracmln.mln.util import ifNone, colorize, out, headline
from pracmln.utils.config import global_config_filename
from pracmln.utils.project import MLNProject, PRACMLNConfig
from pracmln.utils.widgets import FileEditBar


logger = praclog.logger(__name__)

DEFAULTNAME = 'unknown{}'
PRAC_HOME = os.getenv('PRAC_HOME', os.getcwd())
DEFAULT_CONFIG = os.path.join(PRAC_HOME, global_config_filename)
WINDOWTITLE = 'PRAC Query Tool - {}' + os.path.sep + '{}'
WINDOWTITLEEDITED = 'PRAC Query Tool - {}' + os.path.sep + '*{}'


def nop(*args, **kwargs): pass


class PRACQueryGUI(object):
    def __init__(self, master, pracinference, gconf, directory='.'):
        self.master = master

        self.initialized = False

        self.master.bind('<Return>', self.start)
        self.master.bind('<Escape>', lambda a: self.master.quit())
        self.master.protocol('WM_DELETE_WINDOW', self.quit)

        self.prac = pracinference.prac
        self.prac_inference = pracinference
        self.infStep = None

        self.module_dir = os.path.join(os.environ['PRAC_HOME'], 'pracmodules',
                                       'wnsenses')

        self.frame = Frame(master)
        self.frame.pack(fill=BOTH, expand=1)
        self.frame.columnconfigure(1, weight=1)

        # module selection
        row = 0
        Label(self.frame, text="Module: ").grid(row=row, column=0, sticky="E")
        modules = sorted([module for module in self.prac.moduleManifestByName])
        self.selected_module = StringVar(master)
        self.selected_module.trace("w", self.select_module)
        self.list_modules = apply(OptionMenu,
                                  (self.frame, self.selected_module) + tuple(
                                      modules))
        self.list_modules.grid(row=row, column=1, sticky="NWE")

        # Project selection
        row += 1
        Label(self.frame, text="Project: ").grid(row=row, column=0, sticky="E")
        saveprojectcontainer = Frame(self.frame)
        saveprojectcontainer.grid(row=row, column=1, sticky="NEWS")
        saveprojectcontainer.columnconfigure(0, weight=1)

        self.selected_project = StringVar(master)
        projectfiles = ['']
        self.list_projects = apply(OptionMenu, (
        saveprojectcontainer, self.selected_project) + tuple(projectfiles))
        self.list_projects.grid(row=0, column=0, sticky="NWES")
        self.selected_project.trace("w", self.select_project)

        # save proj file
        self.btn_saveproj = Button(saveprojectcontainer,
                                   text='Save Project...',
                                   command=self.noask_save_project)
        self.btn_saveproj.grid(row=0, column=1, sticky="E")

        # save proj file as...
        self.btn_saveproj = Button(saveprojectcontainer,
                                   text='Save Project as...',
                                   command=self.ask_save_project)
        self.btn_saveproj.grid(row=0, column=2, sticky="E")

        # logic selection
        row += 1
        Label(self.frame, text='Logic: ').grid(row=row, column=0, sticky='E')
        logics = ['FirstOrderLogic', 'FuzzyLogic']
        self.selected_logic = StringVar(master)
        self.selected_logic.trace('w', self.settings_setdirty)
        l = apply(OptionMenu,
                  (self.frame, self.selected_logic) + tuple(logics))
        l.grid(row=row, column=1, sticky='NWE')

        # mln section
        row += 1
        Label(self.frame, text="MLN: ").grid(row=row, column=0, sticky='NE')
        self.mln_container = FileEditBar(self.frame, dir=self.module_dir,
                                         filesettings={'extension': '.mln',
                                                       'ftypes': [('MLN files',
                                                                   '.mln')]},
                                         defaultname='*unknown{}',
                                         importhook=self.import_mln,
                                         deletehook=self.delete_mln,
                                         projecthook=self.save_proj,
                                         filecontenthook=self.mlnfilecontent,
                                         fileslisthook=self.mlnfiles,
                                         updatehook=self.update_mln,
                                         onchangehook=self.project_setdirty)
        self.mln_container.grid(row=row, column=1, sticky="NEWS")
        self.mln_container.columnconfigure(1, weight=2)
        self.frame.rowconfigure(row, weight=1)

        row += 1
        self.use_emln = IntVar()
        self.use_emln.set(0)
        self.cb_use_emln = Checkbutton(self.frame, text="use model extension",
                                       variable=self.use_emln,
                                       command=self.onchange_use_emln)
        self.cb_use_emln.grid(row=row, column=1, sticky="W")

        # mln extension section
        row += 1
        self.emlncontainerrow = row
        self.emln_label = Label(self.frame, text="EMLN: ")
        self.emln_label.grid(row=self.emlncontainerrow, column=0, sticky='NE')
        self.emln_container = FileEditBar(self.frame, dir=self.module_dir,
                                          filesettings={'extension': '.emln',
                                                        'ftypes': [(
                                                                   'MLN extension files',
                                                                   '.emln')]},
                                          defaultname='*unknown{}',
                                          importhook=self.import_emln,
                                          deletehook=self.delete_emln,
                                          projecthook=self.save_proj,
                                          filecontenthook=self.emlnfilecontent,
                                          fileslisthook=self.emlnfiles,
                                          updatehook=self.update_emln,
                                          onchangehook=self.project_setdirty)
        self.emln_container.grid(row=self.emlncontainerrow, column=1,
                                 sticky="NEWS")
        self.emln_container.columnconfigure(1, weight=2)
        self.onchange_use_emln(dirty=False)
        self.frame.rowconfigure(row, weight=1)

        # db section
        row += 1
        Label(self.frame, text="Evidence: ").grid(row=row, column=0,
                                                  sticky='NE')
        self.db_container = FileEditBar(self.frame, dir=self.module_dir,
                                        filesettings={'extension': '.db',
                                                      'ftypes': [(
                                                                 'Database files',
                                                                 '.db')]},
                                        defaultname='*unknown{}',
                                        importhook=self.import_db,
                                        deletehook=self.delete_db,
                                        projecthook=self.save_proj,
                                        filecontenthook=self.dbfilecontent,
                                        fileslisthook=self.dbfiles,
                                        updatehook=self.update_db,
                                        onchangehook=self.project_setdirty)
        self.db_container.grid(row=row, column=1, sticky="NEWS")
        self.db_container.columnconfigure(1, weight=2)
        self.frame.rowconfigure(row, weight=1)

        # inference method selection
        row += 1
        self.list_methods_row = row
        Label(self.frame, text="Method: ").grid(row=row, column=0, sticky=E)
        self.selected_method = StringVar(master)
        self.selected_method.trace('w', self.settings_setdirty)
        self.list_methods = OptionMenu(self.frame, self.selected_method,
                                       *InferenceMethods.names())
        self.list_methods.grid(row=self.list_methods_row, column=1,
                               sticky="NWE")

        # queries
        row += 1
        Label(self.frame, text="Queries: ").grid(row=row, column=0, sticky=E)
        self.query = StringVar(master)
        Entry(self.frame, textvariable=self.query).grid(row=row, column=1,
                                                        sticky="NEW")

        #  parameters
        row += 1
        Label(self.frame, text="Parameters: ").grid(row=row, column=0,
                                                    sticky="NE")
        self.params = StringVar(master)
        self.entry_params = Entry(self.frame, textvariable=self.params)
        self.entry_params.grid(row=row, column=1, sticky="NEW")

        # closed-world predicates
        row += 1
        Label(self.frame, text="CW preds: ").grid(row=row, column=0,
                                                  sticky="NE")
        self.cwpreds = StringVar(master)
        self.entry_cw = Entry(self.frame, textvariable=self.cwpreds)
        self.entry_cw.grid(row=row, column=1, sticky="NEW")

        # all preds open-world
        cw_container = Frame(self.frame)
        cw_container.grid(row=row, column=1, sticky="NES")
        self.closed_world = IntVar()
        self.cb_closed_world = Checkbutton(cw_container,
                                           text="Apply CW assumption to all except queries",
                                           variable=self.closed_world)
        self.cb_closed_world.grid(row=row, column=2, sticky='E')

        # Multiprocessing and verbose
        row += 1
        options_container = Frame(self.frame)
        options_container.grid(row=row, column=1, sticky='NEWS')

        self.multicore = IntVar()
        self.cb_multicore = Checkbutton(options_container, text="Use all CPUs",
                                        variable=self.multicore)
        self.cb_multicore.grid(row=0, column=0, sticky=W)

        self.verbose = IntVar()
        self.cb_verbose = Checkbutton(options_container, text="verbose",
                                      variable=self.verbose)
        self.cb_verbose.grid(row=0, column=1, sticky=W)

        self.keep_evidence = IntVar()
        self.cb_keep_evidence = Checkbutton(options_container,
                                            text="keep result",
                                            variable=self.keep_evidence)
        self.cb_keep_evidence.grid(row=0, column=2, sticky=W)
        self.keep_evidence.set(True)

        # start and continue buttons
        row += 1
        self.btn_container = Frame(self.frame)
        self.btn_container.grid(row=row, column=1, sticky='EW')

        start_button = Button(self.btn_container, text="Start Inference",
                              command=self.start)
        start_button.grid(row=0, column=1, sticky='E')

        continue_button = Button(self.btn_container, text="Continue >",
                                 command=self.oncontinue)
        continue_button.grid(row=0, column=2, sticky='W')

        self.settings_dirty = IntVar()
        self.project_dirty = IntVar()

        self.gconf = gconf
        self.project = None
        self.dir = os.path.abspath(
            ifNone(gconf['prev_query_path'], DEFAULT_CONFIG))
        if gconf['prev_query_project': self.dir] is not None:
            self.load_project(
                os.path.join(self.dir, gconf['prev_query_project': self.dir]))
        else:
            self.new_project()

        self.config = self.project.queryconf
        self.project.addlistener(self.project_setdirty)

        self.selected_module.set(self.gconf.get("module", modules[0]))
        self.update_dbeditor_from_result(
            *pracinference.inference_steps[-1].output_dbs)
        self.mln_container.dirty = False
        self.emln_container.dirty = False
        self.db_container.dirty = False
        self.project_setdirty(dirty=False)

        self.master.geometry(gconf['window_loc_query'])

        self.initialized = True


    def quit(self):
        if self.settings_dirty.get() or self.project_dirty.get():
            savechanges = tkMessageBox.askyesnocancel("Save changes",
                                                      "You have unsaved project changes. Do you want to save them before quitting?")
            if savechanges is None:
                return
            elif savechanges:
                self.noask_save_project()
            self.master.destroy()
        else:
            # write gui settings and destroy
            self.write_gconfig()
            self.master.destroy()


    ####################### PROJECT FUNCTIONS #################################
    def new_project(self):
        self.project = MLNProject()
        self.project.addlistener(self.project_setdirty)
        self.project.name = DEFAULTNAME.format('.pracmln')
        self.reset_gui()
        self.set_config(self.project.queryconf)
        self.mln_container.update_file_choices()
        self.emln_container.update_file_choices()
        self.db_container.update_file_choices()
        self.settings_setdirty()


    def project_setdirty(self, dirty=False, *args):
        self.project_dirty.set(
            dirty or self.mln_container.dirty or self.db_container.dirty or self.emln_container.dirty)
        self.changewindowtitle()


    def settings_setdirty(self, *args):
        self.settings_dirty.set(1)
        self.changewindowtitle()


    def changewindowtitle(self):
        title = (WINDOWTITLEEDITED if (
        self.settings_dirty.get() or self.project_dirty.get()) else WINDOWTITLE).format(
            self.dir, self.project.name)
        self.master.title(title)


    def select_project(self, *args):
        filename = os.path.join(self.prac.moduleManifestByName[
                                    self.selected_module.get()].module_path,
                                self.selected_project.get())
        if filename and os.path.exists(filename):
            self.load_project(filename)
        else:
            logger.info('No file selected.')
            return


    def load_project(self, filename):
        if filename and os.path.exists(filename):
            projdir, _ = ntpath.split(filename)
            self.dir = os.path.abspath(projdir)
            self.module_dir = os.path.abspath(projdir)
            self.project = MLNProject.open(filename)
            self.project.addlistener(self.project_setdirty)
            self.reset_gui(keepdb=True)
            self.set_config(self.project.queryconf.config)
            self.mln_container.update_file_choices()
            self.db_container.update_file_choices()
            if len(self.project.mlns) > 0:
                self.mln_container.selected_file.set(
                    self.project.queryconf['mln'] or self.project.mlns.keys()[
                        0])
                self.mln_container.dirty = False
            if len(self.project.emlns) > 0:
                self.emln_container.selected_file.set(
                    self.project.queryconf['emln'] or
                    self.project.emlns.keys()[0])
                self.emln_container.dirty = False
            if len(self.project.dbs) > 0 and not self.keep_evidence.get():
                self.db_container.selected_file.set(
                    self.project.queryconf['db'] or self.project.dbs.keys()[0])
            self.write_gconfig(savegeometry=False)
            self.settings_dirty.set(0)
            self.changewindowtitle()
        else:
            logger.error(
                'File {} does not exist. Creating new project...'.format(
                    filename))
            self.new_project()


    def noask_save_project(self):
        if self.project.name and not self.project.name == DEFAULTNAME.format(
                '.pracmln'):
            self.save_project(os.path.join(self.module_dir, self.project.name))
        else:
            self.ask_save_project()


    def ask_save_project(self):
        fullfilename = asksaveasfilename(initialdir=self.module_dir,
                                         confirmoverwrite=True, filetypes=[
                ('PRACMLN project files', '.pracmln')],
                                         defaultextension=".pracmln")
        self.save_project(fullfilename)


    def save_project(self, fullfilename):
        if fullfilename is not None:
            fpath, fname = ntpath.split(fullfilename)
            fname = fname.split('.')[0]
            self.project.name = fname
            self.dir = os.path.abspath(fpath)
            self.module_dir = os.path.abspath(fpath)

            self.mln_container.save_all_files()
            self.emln_container.save_all_files()
            self.db_container.save_all_files()

            self.update_config()
            self.project.save(dirpath=self.module_dir)
            self.write_gconfig()

            self.load_project(fullfilename)
            self.settings_dirty.set(0)


    def save_proj(self):
        self.project.save(dirpath=self.module_dir)
        self.write_gconfig()
        self.project_setdirty()


    ####################### MLN FUNCTIONS #####################################
    def import_mln(self, name, content):
        self.project.add_mln(name, content)


    def delete_mln(self, fname):
        if fname in self.project.mlns:
            self.project.rm_mln(fname)
        fnamestr = fname.strip('*')
        if fnamestr in self.project.mlns:
            self.project.rm_mln(fnamestr)


    def update_mln(self, old=None, new=None, content=None, askoverwrite=True):
        if old is None:
            old = self.mln_container.selected_file.get()
        if new is None:
            new = self.mln_container.selected_file.get().strip('*')
        if content is None:
            content = self.mln_container.editor.get("1.0", END).strip()

        if old == new and askoverwrite:
            savechanges = tkMessageBox.askyesno("Save changes",
                                                "A file '{}' already exists. Overwrite?".format(
                                                    new))
            if savechanges:
                self.project.mlns[old] = content
            else:
                logger.error('no name specified!')
                return -1
        elif old == new and not askoverwrite:
            self.project.mlns[old] = content
        else:
            if new in self.project.mlns:
                if askoverwrite:
                    savechanges = tkMessageBox.askyesno("Save changes",
                                                        "A file '{}' already exists. Overwrite?".format(
                                                            new))
                    if savechanges:
                        self.project.mlns[new] = content
                    else:
                        logger.error('no name specified!')
                        return -1
            else:
                self.project.mlns[new] = content
        return 1


    def mlnfiles(self):
        return self.project.mlns.keys()


    def mlnfilecontent(self, filename):
        return self.project.mlns.get(filename, '').strip()


    ####################### /MLN FUNCTIONS #####################################


    ####################### EMLN FUNCTIONS #####################################
    def import_emln(self, name, content):
        self.project.add_emln(name, content)


    def delete_emln(self, fname):
        if fname in self.project.emlns:
            self.project.rm_emln(fname)
        fnamestr = fname.strip('*')
        if fnamestr in self.project.emlns:
            self.project.rm_emln(fnamestr)


    def update_emln(self, old=None, new=None, content=None, askoverwrite=True):
        if old is None:
            old = self.emln_container.selected_file.get()
        if new is None:
            new = self.emln_container.selected_file.get().strip('*')
        if content is None:
            content = self.emln_container.editor.get("1.0", END).strip()

        if old == new and askoverwrite:
            savechanges = tkMessageBox.askyesno("Save changes",
                                                "A file '{}' already exists. Overwrite?".format(
                                                    new))
            if savechanges:
                self.project.emlns[old] = content
            else:
                logger.error('no name specified!')
                return -1
        elif old == new and not askoverwrite:
            self.project.emlns[old] = content
        else:
            if new in self.project.emlns:
                if askoverwrite:
                    savechanges = tkMessageBox.askyesno("Save changes",
                                                        "A file '{}' already exists. Overwrite?".format(
                                                            new))
                    if savechanges:
                        self.project.emlns[new] = content
                    else:
                        logger.error('no name specified!')
                        return -1
            else:
                self.project.emlns[new] = content
        return 1


    def emlnfiles(self):
        return self.project.emlns.keys()


    def emlnfilecontent(self, filename):
        return self.project.emlns.get(filename, '').strip()


    ####################### /EMLN FUNCTIONS #####################################


    ####################### DB FUNCTIONS #####################################
    def import_db(self, name, content):
        self.project.add_db(name, content)


    def delete_db(self, fname):
        if fname in self.project.dbs:
            self.project.rm_db(fname)
        fnamestr = fname.strip('*')
        if fnamestr in self.project.dbs:
            self.project.rm_db(fnamestr)


    def update_db(self, old=None, new=None, content=None, askoverwrite=True):
        if old is None:
            old = self.db_container.selected_file.get()
        if new is None:
            new = self.db_container.selected_file.get().strip('*')
        if content is None:
            content = self.db_container.editor.get("1.0", END).strip()

        if old == new and askoverwrite:
            savechanges = tkMessageBox.askyesno("Save changes",
                                                "A file '{}' already exists. Overwrite?".format(
                                                    new))
            if savechanges:
                self.project.dbs[old] = content
            else:
                logger.error('no name specified!')
                return -1
        elif old == new and not askoverwrite:
            self.project.dbs[old] = content
        else:
            if new in self.project.dbs:
                if askoverwrite:
                    savechanges = tkMessageBox.askyesno("Save changes",
                                                        "A file '{}' already exists. Overwrite?".format(
                                                            new))
                    if savechanges:
                        self.project.dbs[new] = content
                    else:
                        logger.error('no name specified!')
                        return -1
            else:
                self.project.dbs[new] = content
        return 1


    def dbfiles(self):
        return self.project.dbs.keys()


    def dbfilecontent(self, filename):
        return self.project.dbs.get(filename, '').strip()


    ####################### /DB FUNCTIONS #####################################


    ####################### GENERAL FUNCTIONS #################################

    def onchange_use_emln(self, dirty=True, *args):
        if not self.use_emln.get():
            self.emln_label.grid_forget()
            self.emln_container.grid_forget()
        else:
            self.emln_label.grid(row=self.emlncontainerrow, column=0,
                                 sticky="NE")
            self.emln_container.grid(row=self.emlncontainerrow, column=1,
                                     sticky="NWES")
        if dirty:
            self.settings_setdirty()


    def onchange_cw(self, *args):
        if self.closed_world.get():
            self.entry_cw.configure(state=DISABLED)
        else:
            self.entry_cw.configure(state=NORMAL)
        self.settings_setdirty()


    def reset_gui(self, keepdb=False):
        self.set_config({})
        self.db_container.clear(keep=keepdb)
        self.emln_container.clear()
        self.mln_container.clear()


    def set_config(self, conf):
        self.config = conf
        self.selected_logic.set(ifNone(conf.get('logic'), 'FirstOrderLogic'))
        self.mln_container.selected_file.set(ifNone(conf.get('mln'), ''))
        if not self.keep_evidence:
            self.db_container.selected_file.set(ifNone(conf.get('db'), ''))
        self.selected_method.set(
            ifNone(self.config.get("method"), InferenceMethods.name('MCSAT'),
                   transform=InferenceMethods.name))
        self.use_emln.set(ifNone(conf.get('use_emln'), False))
        if self.use_emln.get():
            self.emln_container.selected_file.set(ifNone(conf.get('emln'), ""))
        self.multicore.set(ifNone(conf.get('multicore'), False))
        self.verbose.set(ifNone(conf.get('verbose'), False))
        self.params.set(ifNone(conf.get('params'), ''))
        self.cwpreds.set(ifNone(conf.get('cw_preds'), ''))
        self.closed_world.set(ifNone(conf.get('cw'), False))
        self.query.set(ifNone(conf.get('queries'), 'foo, bar'))


    def update_config(self):

        self.config = PRACMLNConfig()
        self.config[
            'mln'] = self.mln_container.selected_file.get().strip().lstrip('*')
        self.config[
            'emln'] = self.emln_container.selected_file.get().strip().lstrip(
            '*')
        self.config[
            "db"] = self.db_container.selected_file.get().strip().lstrip('*')
        self.config["method"] = InferenceMethods.id(
            self.selected_method.get().strip())
        self.config["params"] = self.params.get().strip()
        self.config["queries"] = self.query.get()
        self.config["cw"] = self.closed_world.get()
        self.config["cw_preds"] = self.cwpreds.get()
        self.config["use_emln"] = self.use_emln.get()
        self.config['logic'] = self.selected_logic.get()
        self.config['multicore'] = self.multicore.get()
        self.config['verbose'] = self.verbose.get()
        self.config['window_loc'] = self.master.winfo_geometry()
        self.config['dir'] = self.dir
        self.project.queryconf = PRACMLNConfig()
        self.project.queryconf.update(self.config.config.copy())


    def write_gconfig(self, savegeometry=True):
        self.gconf['prev_query_path'] = self.dir
        self.gconf['prev_query_project': self.dir] = self.project.name

        # save geometry
        if savegeometry:
            self.gconf['window_loc_query'] = self.master.geometry()
        self.gconf.dump()


    def select_module(self, *args):
        module = self.selected_module.get()
        out('selected module:', module)
        module_path = self.prac.moduleManifestByName[module].module_path
        dirpath = os.path.abspath(module_path)
        self.module_dir = dirpath
        self.update_project_choices()
        if self.gconf['prev_query_project': dirpath] is not None:
            self.selected_project.set(
                self.gconf['prev_query_project': dirpath])


    def update_project_choices(self):
        modulename = self.selected_module.get()
        module_path = self.prac.moduleManifestByName[modulename].module_path

        # remove all items
        self.list_projects['menu'].delete(0, 'end')

        projects = [y for y in os.listdir(module_path) if os.path.isfile(
            os.path.join(module_path, y)) and y.endswith('.pracmln')]
        for p in projects:
            self.list_projects['menu'].add_command(label=p, command=_setit(
                self.selected_project, p))


    def oncontinue(self):
        if self.infStep is None:
            self.start()
        self.prac_inference.inference_steps.append(self.infStep)
        self.update_dbeditor_from_result(*self.infStep.output_dbs)
        logger.info(
            'Input databases have been replaced by the latest results.')


    def update_dbeditor_from_result(self, *dbs):
        self.db_container.selected_file.set('')
        strbuf = StringIO.StringIO()
        for i, db in enumerate(dbs):
            db.write(strbuf, bars=False, color=False)
            if i < len(dbs) - 1:
                strbuf.write('---\n')

        self.db_container.editor.delete("1.0", END)
        self.db_container.editor.insert(INSERT,
                                        strbuf.getvalue().encode('utf-8'))
        strbuf.close()
        self.db_container._editor_dirty = True


    def update_result_from_dbeditor(self):
        dbtext = self.db_container.editor.get("1.0", END).encode(
            'utf8').strip()
        dbobj = parse_db(self.prac.mln, dbtext,
                         ignore_unknown_preds=self.config.get(
                             'ignore_unknown_preds', True))

        self.prac_inference.inference_steps[-1].output_dbs = dbobj


    def start(self, savegeometry=True):

        # create conf from current gui settings
        self.update_config()
        self.update_mln(askoverwrite=False)

        # write gui settings
        self.write_gconfig(savegeometry=savegeometry)

        # hide gui
        self.master.withdraw()

        # if evidence was modified in gui, update dbs for next inference step
        self.update_result_from_dbeditor()

        # runinference
        try:
            print headline('PRAC QUERY TOOL')
            print

            module = self.prac.getModuleByName(self.selected_module.get())

            self.infStep = module(self.prac_inference,
                                  project=self.project,
                                  projectpath=os.path.join(self.module_dir,
                                                           self.project.name))

        except:
            cls, e, tb = sys.exc_info()
            sys.stderr.write("Error: %s\n" % str(e))
            traceback.print_tb(tb)

        # restore main window
        self.master.deiconify()


if __name__ == '__main__':

    from optparse import OptionParser


    parser = OptionParser()
    parser.add_option("-i", "--interactive", dest="interactive", default=False,
                      action='store_true',
                      help="Starts PRAC inference with an interactive GUI tool.")
    (options, args) = parser.parse_args()

    sentences = args
    prac = PRAC()
    prac.wordnet = WordNet(concepts=None)

    root = Tk()
    inference = PRACInference(prac, sentences)
    conf = PRACMLNConfig(DEFAULT_CONFIG)

    if options.interactive:  # use the GUI
        # in case we have natural-language parameters, parse them
        if len(inference.instructions) > 0:
            parser = prac.getModuleByName('nl_parsing')
            prac.run(inference, parser)
            #Started control structure handling
            '''
            cs_recognition = prac.getModuleByName('cs_recognition')
            prac.run(inference, cs_recognition)
            
            
            dbs = inference.inference_steps[-1].output_dbs
            dbs_ = []
            
            for db in dbs:
                dbs_.extend(parser.extract_multiple_action_cores(db)) 
            inference.inference_steps[-1].output_dbs = dbs_
            '''
        app = PRACQueryGUI(root, inference, conf, directory=args[0] if args else None)
        root.mainloop()
    else: # regular PRAC pipeline
        
        while inference.next_module() != None :
            modulename = inference.next_module()
            module = prac.getModuleByName(modulename)
            prac.run(inference, module)

        print
        print colorize('+========================+', (None, 'green', True),
                       True)
        print colorize('| PRAC INFERENCE RESULTS |', (None, 'green', True),
                       True)
        print colorize('+========================+', (None, 'green', True),
                       True)

        step = inference.inference_steps[-1]
        wordnet_module = prac.getModuleByName('wn_senses')
        for db in step.output_dbs:
            for a in sorted(db.evidence.keys()):
                v = db.evidence[a]
                if v > 0.001 and (a.startswith('action_core') or a.startswith(
                        'has_sense') or a.startswith('achieved_by')):
                    if a.startswith('has_sense'):

                        group = re.split(',',
                                         re.split('has_sense\w*\(|\)', a)[1])
                        word = group[0];
                        sense = group[1];
                        if sense != 'null':
                            print
                            print colorize('  WORD:', (None, 'white', True),
                                           True), word,
                            print colorize('  SENSE:', (None, 'white', True),
                                           True), sense
                            wordnet_module.printWordSenses(
                                wordnet_module.get_possible_meanings_of_word(
                                    db, word), sense)
                            print
                    else:
                        print '%.3f    %s' % (v, a)
            RolequeryHandler.queryRolesBasedOnActioncore(db).write(color=True)

    if hasattr(inference.inference_steps[-1], 'executable_plans'):
        print
        print colorize('+==========================+', (None, 'green', True),
                       True)
        print colorize('| PARAMETERIZED ROBOT PLAN |', (None, 'green', True),
                       True)
        print colorize('+==========================+', (None, 'green', True),
                       True)
        print
        for plan in step.executable_plans:
            print plan
