import pyjd # dummy in pyjs

from pyjamas.ui.RootPanel import RootPanel
from pyjamas.ui.HTML import HTML
#from pyjamas.ui.NamedFrame import NamedFrame
#from pyjamas.ui.Hyperlink import Hyperlink
from pyjamas.ui.DockPanel import DockPanel
from pyjamas.ui import HasAlignment
from pyjamas.ui.VerticalPanel import VerticalPanel
from pyjamas.ui.ScrollPanel import ScrollPanel
from pyjamas import Window
from SinkList import SinkList
from pyjamas import History
import Chapter
from pyjamas.HTTPRequest import HTTPRequest
from BookLoader import ChapterListLoader
#from pyjamas.ui.vertsplitpanel import VerticalSplitPanel

def loadSection(section):

    chapter = Chapter.Chapter()
    chapter.setStyleName("ks-Sink")
    chapter.name = section
    RootPanel().add(chapter)
    chapter.onShow()

class Bookreader:

    def onHistoryChanged(self, token):
        info = self.sink_list.find(token)
        if info:
            self.show(info, False)
        else:
            self.showInfo()

    def onModuleLoad(self):
        section = Window.getLocation().getSearchVar("section")
        if not section:
            self.loadChapters()
        else:
            loadSection(section)

    def loadChapters(self):

        self.curInfo = ''
        self.curSink = None
        self.description = HTML()
        self.sink_list = SinkList()
        self.panel = DockPanel()
        
        self.loadSinks()
        self.sinkContainer = DockPanel()
        self.sinkContainer.setStyleName("ks-Sink")

        #self.nf = NamedFrame("section")
        #self.nf.setWidth("100%")
        #self.nf.setHeight("10000")

        self.sp = ScrollPanel(self.sinkContainer)
        #self.sp = VerticalSplitPanel()
        self.sp.setWidth("100%")
        self.sp.setHeight("100%")

        #self.sp.setTopWidget(self.sinkContainer)
        #self.sp.setBottomWidget(self.nf)
        #self.sp.setSplitPosition(10000) # deliberately high - max out.

        vp = VerticalPanel()
        vp.setWidth("99%")
        vp.setHeight("100%")
        vp.add(self.description)
        vp.add(self.sp)

        authors = [
            ("2008, 2009", "Kenneth Casson Leighton", "lkcl@lkcl.net")
        ]
        for years, name, email in authors:
            authors_html = \
            '&copy; %s <a href="mailto:%s">%s</a><br />' %\
            (years, email, name)
        authors_panel = HTML()
        authors_panel.setStyleName("ks-Authors")
        authors_panel.setHTML(authors_html[:-6])

        left_panel = DockPanel(Height="100%")
        left_panel.add(self.sink_list, DockPanel.NORTH)
        left_panel.add(authors_panel, DockPanel.SOUTH)

        self.description.setStyleName("ks-Intro")

        self.panel.add(left_panel, DockPanel.WEST)
        self.panel.add(vp, DockPanel.CENTER)

        self.panel.setCellVerticalAlignment(self.sink_list,
                                            HasAlignment.ALIGN_TOP)
        self.panel.setCellWidth(vp, "100%")
        self.panel.setCellHeight(vp, "100%")

        Window.addWindowResizeListener(self)

        History.addHistoryListener(self)
        RootPanel().add(self.panel)

        self.onWindowResized(Window.getClientWidth(), Window.getClientHeight())

    def onWindowResized(self, width, height):
        self.panel.setWidth(width-20)
        self.sink_list.resize(width-20, height-130)
        self.sp.setHeight("%dpx" % (height-130))

    def show(self, info, affectHistory):
        if info == self.curInfo:
            return
        self.curInfo = info

        #Logger.write("showing " + info.getName())
        if self.curSink is not None:
            self.curSink.onHide()
            #Logger.write("removing " + self.curSink)
            self.sinkContainer.remove(self.curSink)

        self.curSink = info.getInstance()
        self.sink_list.setSinkSelection(info.getName())
        self.sink_list.sp.setScrollPosition(0)
        self.sink_list.sp.setHorizontalScrollPosition(0)
        self.description.setHTML(info.getDescription())

        if (affectHistory):
            History().newItem(info.getName())

        self.sinkContainer.add(self.curSink, DockPanel.CENTER)
        self.sinkContainer.setCellWidth(self.curSink, "100%")
        self.sinkContainer.setCellHeight(self.curSink, "100%")
        self.sinkContainer.setCellVerticalAlignment(self.curSink,
                                                    HasAlignment.ALIGN_TOP)
        self.curSink.onShow()
        
    def loadSinks(self):
        HTTPRequest().asyncGet("contents.txt", ChapterListLoader(self))

    def setChapters(self, chapters):
        for l in chapters:
            name = l[0]
            desc = l[1]
            self.sink_list.addSink(Chapter.init(name, desc))

        #Show the initial screen.
        initToken = History.getToken()
        if len(initToken):
            self.onHistoryChanged(initToken)
        else:
            self.showInfo()


    def showInfo(self):
        self.show(self.sink_list.sinks[0], False)


if __name__ == '__main__':
    pyjd.setup("http://127.0.0.1/pyjamas/doc/book/public/Bookreader.html")
    app = Bookreader()
    app.onModuleLoad()
    pyjd.run()
