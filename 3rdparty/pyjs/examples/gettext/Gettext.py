import pyjd # this is dummy in pyjs.
from pyjamas.ui.RootPanel import RootPanel
from pyjamas.ui.Button import Button
from pyjamas.ui.HTML import HTML
from pyjamas.ui.Label import Label
from pyjamas import Window
from i18n import i18n, _, ngettext
import pygwt


lang = [
    'nl_NL',
    'de_DE',
    'en_US',
]


class GettextExample(object):

    def __init__(self):
        self.b = Button(_("Click me"), self.greet, StyleName='teststyle')
        self.h = HTML(_("<b>Hello World</b> (html)"), StyleName='teststyle')
        self.l = Label(_("Hello World (label)"), StyleName='teststyle')
        self.base = HTML(_("Hello from %s") % pygwt.getModuleBaseURL(),
                         StyleName='teststyle')
        RootPanel().add(self.b)
        RootPanel().add(self.h)
        RootPanel().add(self.l)
        RootPanel().add(self.base)

    def change_texts(self, text):
        self.b.setText(_("Click me"))
        self.h.setHTML(_("<b>Hello World</b> (html)"))
        self.l.setText(_("Hello World (label)"))
        text = [_("Hello from %s") % pygwt.getModuleBaseURL()]
        for i in range(4):
            text.append(ngettext('%(num)d single', '%(num)d plural', i) % dict(num=i))
        text = '<br />'.join(text)
        self.base.setHTML(text)

    def greet(self, fred):
        fred.setText(_("No, really click me!"))
        Window.alert(_("Hello, there!"))
        i18n.load(lang=lang[0], onCompletion=self.change_texts)
        lang.append(lang.pop(0))

if __name__ == '__main__':
    pyjd.setup("public/Hello.html?fred=foo#me")
    ge = GettextExample()
    pyjd.run()
