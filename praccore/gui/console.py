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
from urwid.treetools import TreeListBox

#!/usr/bin/python
#
# Urwid example lazy directory browser / tree view
#    Copyright (C) 2004-2011  Ian Ward
#    Copyright (C) 2010  Kirk McDonald
#    Copyright (C) 2010  Rob Lanphier
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

"""
Urwid example lazy directory browser / tree view

Features:
- custom selectable widgets for files and directories
- custom message widgets to identify access errors and empty directories
- custom list walker for displaying widgets in a tree fashion
- outputs a quoted list of files and directories "selected" on exit
"""

import itertools
import re
import os

import urwid


class MyTreeListBox(TreeListBox):
    """A ListBox with special handling for navigation and
    collapsing of TreeWidgets"""

    def __init__(self, walker, parent):
        TreeListBox.__init__(self, walker)
        self.parent = parent

    def keypress(self, size, key):
        key = self.__super.keypress(size, key)
        return self.unhandled_input(size, key)

    def unhandled_input(self, size, input):
        """Handle macro-navigation keys"""
        if input == 'left':
            if self.get_focus()[0].expanded:
                self.keypress(size, "-")
            else:
                self.move_focus_to_parent(size)
        elif input == '-':
            self.collapse_focus_parent(size)
        elif input == 'home':
            self.focus_home(size)
        elif input == 'end':
            self.focus_end(size)
        elif input == 'enter':
            self.parent.on_enter(*self.get_focus())
        else:
            return input

class FlagFileWidget(urwid.TreeWidget):
    # apply an attribute to the expand/unexpand icons
    unexpanded_icon = urwid.AttrMap(urwid.TreeWidget.unexpanded_icon,
        'dirmark')
    expanded_icon = urwid.AttrMap(urwid.TreeWidget.expanded_icon,
        'dirmark')

    def __init__(self, node):
        self.__super.__init__(node)
        # insert an extra AttrWrap for our own use
        self._w = urwid.AttrWrap(self._w, None)
        self.flagged = False
        self.update_w()

    def selectable(self):
        return True

    def keypress(self, size, key):
        """allow subclasses to intercept keystrokes"""
        key = self.__super.keypress(size, key)
        if key:
            key = self.unhandled_keys(size, key)
        return key

    def unhandled_keys(self, size, key):
        """
        Override this method to intercept keystrokes in subclasses.
        Default behavior: Toggle flagged on space, ignore other keys.
        """
        if key == " ":
            self.flagged = not self.flagged
            self.update_w()
        else:
            return key

    def update_w(self):
        """Update the attributes of self.widget based on self.flagged.
        """
        if self.flagged:
            self._w.attr = 'flagged'
            self._w.focus_attr = 'flagged focus'
        else:
            self._w.attr = 'body'
            self._w.focus_attr = 'focus'


class FileTreeWidget(FlagFileWidget):
    """Widget for individual files."""
    def __init__(self, node):
        self.__super.__init__(node)
        path = node.get_value()
        add_widget(path, self)

    def get_display_text(self):
        return self.get_node().get_key()
    


class EmptyWidget(urwid.TreeWidget):
    """A marker for expanded directories with no contents."""
    def get_display_text(self):
        return ('flag', '(empty directory)')


class ErrorWidget(urwid.TreeWidget):
    """A marker for errors reading directories."""

    def get_display_text(self):
        return ('error', "(error/permission denied)")


class DirectoryWidget(FlagFileWidget):
    """Widget for a directory."""
    def __init__(self, node):
        self.__super.__init__(node)
        path = node.get_value()
        add_widget(path, self)
        self.expanded = starts_expanded(path)
        self.update_expanded_icon()

    def get_display_text(self):
        node = self.get_node()
#         if node.get_depth() == 0:
#             return "/"
#         else:
        return node.get_key() + ' (%d)' % node.get_depth()


class FileNode(urwid.TreeNode):
    """Metadata storage for individual files"""

    def __init__(self, path, root, parent=None):
        self.root = root
        depth = path.count(dir_sep()) - root.count(dir_sep())
#         print 'creating file node: %s %d' % (path, depth)
        key = os.path.basename(path)
        self.path = path
        urwid.TreeNode.__init__(self, path, key=key, parent=parent, depth=depth)

    def load_parent(self):
        parentname, myname = os.path.split(self.get_value())
        parent = DirectoryNode(parentname, self.root)
        parent.set_child_node(self.get_key(), self)
        return parent

    def load_widget(self):
        return FileTreeWidget(self)


class EmptyNode(urwid.TreeNode):
    def load_widget(self):
        return EmptyWidget(self)


class ErrorNode(urwid.TreeNode):
    def load_widget(self):
        return ErrorWidget(self)


class DirectoryNode(urwid.ParentNode):
    """Metadata storage for directories"""

    def __init__(self, path, root, parent=None):
        self.root = root
        print root, path
        depth = path.count(dir_sep()) - root.count(dir_sep())
        key = os.path.basename(path)
#         print 'depth %d for %s' % (depth, path)
        urwid.ParentNode.__init__(self, path, key=key, parent=parent, depth=depth)

    def load_parent(self):
#         print ';load parent'
        parentname, myname = os.path.split(self.get_value())
        if self.root == self.get_value():
            parentname = self.get_value()
#         print 'parent of %s: %s' % (myname, parentname)
        parent = DirectoryNode(parentname, self.root)
        parent.set_child_node(self.get_key(), self)
        return parent

    def load_child_keys(self):
        dirs = []
        files = []
        try:
            path = self.get_value()
            # separate dirs and files
            for a in os.listdir(path):
                if os.path.isdir(os.path.join(path,a)):
                    dirs.append(a)
                else:
                    files.append(a)
        except OSError, e:
            depth = self.get_depth() + 1
            self._children[None] = ErrorNode(self, parent=self, key=None, 
                                             depth=depth)
            return [None]

        # sort dirs and files
        dirs.sort(key=alphabetize)
        files.sort(key=alphabetize)
        # store where the first file starts
        self.dir_count = len(dirs)
        # collect dirs and files together again
        keys = dirs + files
        if len(keys) == 0:
            depth = self.get_depth() + 1
            self._children[None] = EmptyNode(self, parent=self, key=None, 
                                             depth=depth)
            keys = [None]
        return keys

    def load_child_node(self, key):
        """Return either a FileNode or DirectoryNode"""
        index = self.get_child_index(key)
        if key is None:
            return EmptyNode(None)
        else:
            path = os.path.join(self.get_value(), key)
            if index < self.dir_count:
                return DirectoryNode(path, self.root, parent=self)
            else:
                path = os.path.join(self.get_value(), key)
                return FileNode(path, self.root, parent=self)

    def load_widget(self):
        return DirectoryWidget(self)


class DirectoryBrowser:
    palette = [
        ('body', 'black', 'light gray'),
        ('flagged', 'black', 'dark green', ('bold','underline')),
        ('focus', 'light gray', 'dark blue', 'standout'),
        ('flagged focus', 'yellow', 'dark cyan', 
                ('bold','standout','underline')),
        ('head', 'yellow', 'black', 'standout'),
        ('foot', 'light gray', 'black'),
        ('key', 'light cyan', 'black','underline'),
        ('title', 'white', 'black', 'bold'),
        ('dirmark', 'black', 'dark cyan', 'bold'),
        ('flag', 'dark gray', 'light gray'),
        ('error', 'dark red', 'light gray'),
        ]
    
    footer_text = [
        ('title', "Directory Browser"), "    ",
        ('key', "UP"), ",", ('key', "DOWN"), ",",
        ('key', "PAGE UP"), ",", ('key', "PAGE DOWN"),
        "  ",
        ('key', "SPACE"), "  ",
        ('key', "+"), ",",
        ('key', "-"), "  ",
        ('key', "LEFT"), "  ",
        ('key', "HOME"), "  ", 
        ('key', "END"), "  ",
        ('key', "Q"),
        ]
    
    
    def __init__(self,):
#         cwd = os.getcwd()
        prac_path = os.getenv('PRAC_HOME')
        cwd = os.path.join(prac_path, 'pracmodules')
        store_initial_cwd(cwd)
        self.header = urwid.Text("PLEASE SELECT THE MARKOV LOGIC NETWORK YOU WANT TO CONTINUE WITH")
        self.listbox = MyTreeListBox(urwid.TreeWalker(DirectoryNode(cwd, cwd)))
        self.listbox.offset_rows = 1
        self.footer = urwid.AttrWrap(urwid.Text(self.footer_text),
            'foot')
        self.view = urwid.Frame(
            urwid.AttrWrap(self.listbox, 'body'), 
            header=urwid.AttrWrap(self.header, 'head'), 
            footer=self.footer)

    def main(self):
        """Run the program."""
        
        self.loop = urwid.MainLoop(self.view, self.palette,
            unhandled_input=self.unhandled_input)
        self.loop.run()
    
        # on exit, write the flagged filenames to the console
        names = [escape_filename_sh(x) for x in get_flagged_names()]
        print " ".join(names)

    def unhandled_input(self, k):
        # update display of focus directory
        if k in ('q','Q'):
            raise urwid.ExitMainLoop()
        elif k == 'enter':
            self.parent.on_enter(*self.get_focus())


# def main():
#     DirectoryBrowser().main()


#######
# global cache of widgets
_widget_cache = {}

def add_widget(path, widget):
    """Add the widget for a given path"""

    _widget_cache[path] = widget

def get_flagged_names():
    """Return a list of all filenames marked as flagged."""
    
    l = []
    for w in _widget_cache.values():
        if w.flagged:
            l.append(w.get_node().get_value())
    return l
            


######
# store path components of initial current working directory
_initial_cwd = []

def store_initial_cwd(name):
    """Store the initial current working directory path components."""
    
    global _initial_cwd
    _initial_cwd = name.split(dir_sep())

def starts_expanded(name):
    """Return True if directory is a parent of initial cwd."""

    if name is '/':
        return True
    
    l = name.split(dir_sep())
    if len(l) > len(_initial_cwd):
        return False
    
    if l != _initial_cwd[:len(l)]:
        return False
    
    return True


def escape_filename_sh(name):
    """Return a hopefully safe shell-escaped version of a filename."""

    # check whether we have unprintable characters
    for ch in name: 
        if ord(ch) < 32: 
            # found one so use the ansi-c escaping
            return escape_filename_sh_ansic(name)
            
    # all printable characters, so return a double-quoted version
    name.replace('\\','\\\\')
    name.replace('"','\\"')
    name.replace('`','\\`')
    name.replace('$','\\$')
    return '"'+name+'"'


def escape_filename_sh_ansic(name):
    """Return an ansi-c shell-escaped version of a filename."""
    
    out =[]
    # gather the escaped characters into a list
    for ch in name:
        if ord(ch) < 32:
            out.append("\\x%02x"% ord(ch))
        elif ch == '\\':
            out.append('\\\\')
        else:
            out.append(ch)
            
    # slap them back together in an ansi-c quote  $'...'
    return "$'" + "".join(out) + "'"

SPLIT_RE = re.compile(r'[a-zA-Z]+|\d+')
def alphabetize(s):
    L = []
    for isdigit, group in itertools.groupby(SPLIT_RE.findall(s), key=lambda x: x.isdigit()):
        if isdigit:
            for n in group:
                L.append(('', int(n)))
        else:
            L.append((''.join(group).lower(), 0))
    return L

def dir_sep():
    """Return the separator used in this os."""
    return getattr(os.path,'sep','/')

class LineWalker(urwid.ListWalker):
    """ListWalker-compatible class for lazily reading file contents."""
    
    def __init__(self, name):
        self.file = open(name)
        self.filename = name
        self.lines = []
        self.focus = 0
    
    def get_focus(self): 
        return self._get_at_pos(self.focus)
    
    def set_focus(self, focus):
        self.focus = focus
        self._modified()
    
    def get_next(self, start_from):
        return self._get_at_pos(start_from + 1)
    
    def get_prev(self, start_from):
        return self._get_at_pos(start_from - 1)

    def read_next_line(self):
        """Read another line from the file."""
        
        next_line = self.file.readline()
        
        if not next_line or next_line[-1:] != '\n':
            # no newline on last line of file
            self.file = None
        else:
            # trim newline characters
            next_line = next_line[:-1]

        expanded = next_line.expandtabs()
        
        edit = urwid.Edit("", expanded, allow_tab=True)
        edit.set_edit_pos(0)
        edit.original_text = next_line
        self.lines.append(edit)

        return next_line
        
    
    def _get_at_pos(self, pos):
        """Return a widget for the line number passed."""
        
        if pos < 0:
            # line 0 is the start of the file, no more above
            return None, None
            
        if len(self.lines) > pos:
            # we have that line so return it
            return self.lines[pos], pos

        if self.file is None:
            # file is closed, so there are no more lines
            return None, None

        assert pos == len(self.lines), "out of order request?"

        self.read_next_line()
        
        return self.lines[-1], pos
    
    def split_focus(self):
        """Divide the focus edit widget at the cursor location."""
        
        focus = self.lines[self.focus]
        pos = focus.edit_pos
        edit = urwid.Edit("",focus.edit_text[pos:], allow_tab=True)
        edit.original_text = ""
        focus.set_edit_text(focus.edit_text[:pos])
        edit.set_edit_pos(0)
        self.lines.insert(self.focus+1, edit)

    def combine_focus_with_prev(self):
        """Combine the focus edit widget with the one above."""

        above, ignore = self.get_prev(self.focus)
        if above is None:
            # already at the top
            return
        
        focus = self.lines[self.focus]
        above.set_edit_pos(len(above.edit_text))
        above.set_edit_text(above.edit_text + focus.edit_text)
        del self.lines[self.focus]
        self.focus -= 1

    def combine_focus_with_next(self):
        """Combine the focus edit widget with the one below."""

        below, ignore = self.get_next(self.focus)
        if below is None:
            # already at bottom
            return
        
        focus = self.lines[self.focus]
        focus.set_edit_text(focus.edit_text + below.edit_text)
        del self.lines[self.focus+1]

if __name__=="__main__": 
    main()
        
