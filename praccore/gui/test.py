#!/usr/bin/python
#
# Urwid graphics example program
#    Copyright (C) 2004-2011  Ian Ward
#
#    This library is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Lesser General Public
#    License as published by the Free Software Foundation; either
#    version 2.1 of the License, or (at your option) any later version.
#
#    This library is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with this library; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Urwid web site: http://excess.org/urwid/
from gui.widgets import LineWalker, DirectoryNode, MyTreeListBox, FileNode
import os

"""
Urwid example demonstrating use of the BarGraph widget and creating a 
floating-window appearance.  Also shows use of alarms to create timed
animation.
"""

import urwid

import math
import time


class MLNView(urwid.WidgetWrap):
    """
    A class responsible for providing the application's interface and
    graph display.
    """
    palette = [
        ('body',         'white',      'black', 'standout'),
        ('header',       'white',      'dark red',   'bold'),
        ('screen edge',  'light blue', 'dark cyan'),
        ('main shadow',  'dark gray',  'black'),
        ('line',         'black',      'light gray', 'standout'),
        ('bg background','light gray', 'black'),
        ('bg 1',         'black',      'dark blue', 'standout'),
        ('bg 1 smooth',  'dark blue',  'black'),
        ('bg 2',         'black',      'dark cyan', 'standout'),
        ('bg 2 smooth',  'dark cyan',  'black'),
        ('button normal','light gray', 'dark blue', 'standout'),
        ('button select','white',      'dark green'),
        ('line',         'black',      'light gray', 'standout'),
        ('pg normal',    'white',      'black', 'standout'),
        ('pg complete',  'white',      'dark magenta'),
        ('pg smooth',     'dark magenta','black')
        ]
        
    graph_samples_per_bar = 10
    graph_num_bars = 5
    graph_offset_per_second = 5
    
    def __init__(self, controller):
        self.controller = controller
        self.started = True
        self.start_time = None
        self.offset = 0
        self.last_offset = None
        self.mln = None
        urwid.WidgetWrap.__init__(self, self.main_window())

    def keypress(self, size, key):
        key = self.__super.keypress(size, key)
        if key == 'esc':
            self.exit_program(self)
        else:
            return key

    def exit_program(self, w):
        raise urwid.ExitMainLoop()

    def graph_controls(self):
        w = urwid.Text("Hello")
        return w
    
    def on_enter(self, item, data):
        if type(data) is FileNode:
            if self.editor.body.filename == data.path:
                self.mln = self.editor.body.filename
                self.exit_program(self)
            else:
                self.editor.body = LineWalker(data.path)
                self.editor._invalidate()

    def main_window(self): 
        # create the tree viewer
        prac_path = os.getenv('PRAC_HOME')
        root = os.path.join(prac_path, 'pracmodules')
        preselect = os.path.join(prac_path, 'pracmodules', 'ac_recognition')
        self.header = urwid.Text("PLEASE SELECT THE MARKOV LOGIC NETWORK YOU WANT TO CONTINUE WITH")
        self.listbox = MyTreeListBox(urwid.TreeWalker(DirectoryNode(preselect, root)), self)
        self.listbox.offset_rows = 1
        self.footer = urwid.AttrWrap(urwid.Text('Press <ENTER> twice to continue with the selected MLN. <ESC> to continue without a selection.'),
            'foot')
        self.tree = urwid.Frame(
            urwid.AttrWrap(self.listbox, 'body'), 
            header=urwid.AttrWrap(self.header, 'head'), 
            footer=self.footer)
        
        # create the separator
        vline = urwid.AttrWrap( urwid.SolidFill(u'\u2502'), 'line')
        
        # create the editor
        self.save_name = '/home/nyga/code/prac/pracmodules/nl_parsing/mln/nl_parsing.mln'
        self.walker = LineWalker(self.save_name) 
        self.editor = urwid.ListBox(self.walker)
        self.footer = urwid.AttrWrap(urwid.Text('Footer'),
            "foot")
        self.view = urwid.Frame(urwid.AttrWrap(self.editor, 'body'),
            footer=self.footer)
        
        w = urwid.Columns([('weight', .5, self.tree), 
                           ('fixed', 1, vline), self.view], dividechars=1, focus_column=0)
        w = urwid.Padding(w,('fixed left',1),('fixed right',0))
        w = urwid.AttrWrap(w,'body')
        w = urwid.LineBox(w)
        w = urwid.AttrWrap(w,'line')
        return w
        

class MLNSelector:
    """
    A class responsible for setting up the model and view and running
    the application.
    """
    
    def __init__(self):
        self.view = MLNView(self)
        

    def run(self):
        self.loop = urwid.MainLoop(self.view, self.view.palette)
        self.loop.run()
        return self.view.mln


if '__main__'==__name__:
    sel = MLNSelector()
    print 'You selected the following MLN: %s' % sel.run()
