# TinyMCE editor dialog that works with pyjd as well as pyjs.
# add the following to the loader file:
# <script type="text/javascript"
#         src="./tinymce/jscripts/tiny_mce/tiny_mce.js"></script>
#
# note: versions of tinymce from 3.0 to at least 3.5b1 have a bug
# where a 2nd editor instance in firefox will flicker, go blank,
# and lose keyboard focus.
#


from pyjamas.ui.TextArea import TextArea
from pyjamas.ui.HTML import HTML
from pyjamas.ui.Button import Button
from pyjamas.ui.DockPanel import DockPanel
from pyjamas.ui.DialogWindow import DialogWindow
from pyjamas.ui import HasAlignment
from pyjamas.ui.HorizontalPanel import HorizontalPanel

from pyjamas import DeferredCommand 
from pyjamas.Timer import Timer
from pyjamas import DOM

from __pyjamas__ import doc

_editor_id = 0

class EditDialogWindow(DialogWindow):

    def __init__(self, app):
        self.app = app
        DialogWindow.__init__(
            self, modal=False,
            minimize=True, maximize=True, close=True,
        )
        self.closeButton = Button("Close", self)
        self.saveButton = Button("Save", self)
        self.setText("Sample DialogWindow with embedded image")
        self.msg = HTML("", True)

        global _editor_id
        _editor_id += 1
        editor_id = "editor%d" % _editor_id

        #self.ht = HTML("", ID=editor_id)
        self.txt = TextArea(Text="", VisibleLines=30, CharacterWidth=80,
                        ID=editor_id)
        dock = DockPanel()
        dock.setSpacing(4)

        hp = HorizontalPanel(Spacing="5")
        hp.add(self.saveButton)
        hp.add(self.closeButton)

        dock.add(hp, DockPanel.SOUTH)
        dock.add(self.msg, DockPanel.NORTH)
        dock.add(self.txt, DockPanel.CENTER)

        dock.setCellHorizontalAlignment(hp, HasAlignment.ALIGN_RIGHT)
        dock.setCellWidth(self.txt, "100%")
        dock.setWidth("100%")
        self.setWidget(dock)
        self.editor_id = editor_id
        self.editor_created = False

    def add_tinymce(self):

        # for storing the results when available
        iframe = DOM.createElement("iframe")
        DOM.setElemAttribute(iframe, "id", "__edit_%s" % self.editor_id)
        DOM.setElemAttribute(iframe, "style", "display:none")
        doc().body.appendChild(iframe)

        # activate tinymce
        new_script = DOM.createElement("script")
        new_script.innerHTML = """
tinyMCE.init({
        // General options
        mode : "textareas",
        theme : "simple",

        // Theme options
        theme_advanced_buttons1 : "bold,italic,underline,strikethrough,|,justifyleft,justifycenter,justifyright,justifyfull,|,styleselect,formatselect,|,table,removeformat",
        theme_advanced_buttons2 : "",
        theme_advanced_buttons3 : "",
        theme_advanced_buttons4 : "",
        theme_advanced_toolbar_location : "top",
        theme_advanced_toolbar_align : "left",
        theme_advanced_statusbar_location : "bottom",
        theme_advanced_resizing : true,
});
"""

        ih = """
var ed = new tinymce.Editor('%s',{
        mode : "none",
        theme : "advanced",
        plugins : "inlinepopups",
        theme_advanced_buttons1 : "bold,italic,underline,strikethrough,|,justifyleft,justifycenter,justifyright,justifyfull,|,formatselect,|,table,image,removeformat",
        theme_advanced_buttons2 : "",
        theme_advanced_buttons3 : "",
        theme_advanced_buttons4 : "",
        theme_advanced_toolbar_location : "top",
        theme_advanced_toolbar_align : "left",
        theme_advanced_statusbar_location : "bottom",
        theme_advanced_resizing : true
});
ed.render();
ed.load();
tinymce.add(ed);
""" % self.editor_id

        print new_script.innerHTML

        DOM.setElemAttribute(new_script, "type", "text/javascript")
        doc().body.appendChild(new_script)

    def load(self, token, fname, data):

        left = 50 # self.fDialogButton.getAbsoluteLeft() + 10
        top = 50 # self.fDialogButton.getAbsoluteTop() + 10
        self.setPopupPosition(left, top)
        self.show()

        self.token = token
        self.fname = fname
        self.msg.setHTML("<center>This is an example of a standard dialog box component.<br>  You can put pretty much anything you like into it,<br>such as the following image '%s':</center>" % fname)
        self.txt.setText(data)

        if not self.editor_created:
            self.editor_created = True
            if self.fname.endswith(".html"):
                Timer(1500, notify=self._load_tinymce)

    def _load_tinymce(self, timer):
        self.add_tinymce()
        #self.load_tinymce()

    def load_tinymce(self):

        # activate tinymce
        new_script = DOM.createElement("script")
        new_script.innerHTML = """
var ed = tinyMCE.get('%s');
ed.init();
ed.render();
""" % self.editor_id

        print new_script.innerHTML

        DOM.setElemAttribute(new_script, "type","text/javascript")
        doc().body.appendChild(new_script)

    def transfer_tinymce(self):

        new_script = DOM.createElement("script")
        new_script.innerHTML = """
var ed = tinyMCE.get('%s');
var data = ed.getContent({'format': 'raw'});
frame = document.getElementById('__edit_%s');
frame.innerText = data;
ed.save();
ed.remove();
""" % (self.editor_id, self.editor_id)

        self.editor_created = False

        DOM.setElemAttribute(new_script, "type","text/javascript")
        doc().body.appendChild(new_script)
        self.hide()
        t = Timer(notify=self)
        t.scheduleRepeating(1000)

    def onTimer(self, timer):

        iframe = doc().getElementById("__edit_%s" % self.editor_id)
        print dir(iframe)
        txt = iframe.innerText
        if not txt:
            return

        timer.cancel()

        doc().body.removeChild(iframe)
        self.app.save_page(self.token, self.fname, txt)

    def onClick(self, sender):
        if sender == self.saveButton:
            if self.fname.endswith(".html"):
                self.transfer_tinymce()
            else:
                txt = self.txt.getText()
                self.app.save_page(self.token, self.fname, txt)
                self.hide()
        else:
            self.hide()

