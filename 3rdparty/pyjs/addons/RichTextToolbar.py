"""
* Copyright 2007 Google Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not
* use this file except in compliance with the License. You may obtain a copy of
* the License at
*
* http:#www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations under
* the License.
"""
#package com.google.gwt.sample.kitchensink.client




from pyjamas import Window
from pyjamas.ui.Image import Image
from pyjamas.ui.ChangeListener import ChangeHandler
from pyjamas.ui.ClickListener import ClickHandler
from pyjamas.ui.Composite import Composite
from pyjamas.ui.HorizontalPanel import HorizontalPanel
from pyjamas.ui.KeyboardListener import KeyboardHandler
from pyjamas.ui.ListBox import ListBox
from pyjamas.ui.PushButton import PushButton
from pyjamas.ui import RichTextArea 
from pyjamas.ui.ToggleButton import ToggleButton
from pyjamas.ui.VerticalPanel import VerticalPanel
from pyjamas.ui.Widget import Widget

class Images(object):
    bold = "bold.gif"
    italic = "italic.gif"
    underline = "underline.gif"
    subscript = "subscript.gif"
    superscript = "superscript.gif"
    justifyLeft = "justifyLeft.gif"
    justifyCenter = "justifyCenter.gif"
    justifyRight = "justifyRight.gif"
    strikeThrough = "strikeThrough.gif"
    indent = "indent.gif"
    outdent = "outdent.gif"
    hr = "hr.gif"
    ol = "ol.gif"
    ul = "ul.gif"
    insertImage = "insertImage.gif"
    createLink = "createLink.gif"
    removeLink = "removeLink.gif"
    removeFormat = "removeFormat.gif"

"""*
* A sample toolbar for use with {@link RichTextArea}. It provides a simple UI
* for all rich text formatting, dynamically displayed only for the available
* functionality.
"""
class RichTextToolbar(Composite, ClickHandler, ChangeHandler, KeyboardHandler):



    fontSizesConstants = [
        RichTextArea.XX_SMALL, RichTextArea.X_SMALL,
        RichTextArea.SMALL, RichTextArea.MEDIUM,
        RichTextArea.LARGE, RichTextArea.X_LARGE,
        RichTextArea.XX_LARGE
    ]

    """*
    * Creates a toolbar that drives the given rich text area.
    *
    * @param richText the rich text area to be controlled
    """
    def __init__(self, richText, **kwargs):

        self.outer = VerticalPanel()
        self.topPanel = HorizontalPanel(BorderWidth=1)
        self.bottomPanel = HorizontalPanel()

        self.richText = richText
        self.basic = richText.getBasicFormatter()
        self.extended = richText.getExtendedFormatter()

        self.outer.add(self.topPanel)
        self.outer.add(self.bottomPanel)
        self.topPanel.setWidth("100%")
        self.topPanel.setHeight("20px")
        self.bottomPanel.setWidth("100%")

        kwargs['StyleName'] = kwargs.get('StyleName', "gwt-RichTextToolbar")

        Composite.__init__(self, self.outer, **kwargs)
        ClickHandler.__init__(self)
        ChangeHandler.__init__(self)
        KeyboardHandler.__init__(self)

        if self.basic is not None:
            self.bold = self.createToggleButton(Images.bold,
                                            "bold")
            self.italic = self.createToggleButton(Images.italic,
                                            "italic")
            self.underline = self.createToggleButton(Images.underline,
                                            "underline")
            self.subscript = self.createToggleButton(Images.subscript,
                                            "subscript")
            self.superscript = self.createToggleButton(Images.superscript,
                                            "superscript")
            self.justifyLeft = self.createPushButton(Images.justifyLeft,
                                            "justify left")
            self.justifyCenter = self.createPushButton(Images.justifyCenter,
                                            "justify centre")
            self.justifyRight = self.createPushButton(Images.justifyRight,
                                            "justify right")
            self.topPanel.add(self.bold)
            self.topPanel.add(self.italic)
            self.topPanel.add(self.underline)
            self.topPanel.add(self.subscript)
            self.topPanel.add(self.superscript)
            self.topPanel.add(self.justifyLeft)
            self.topPanel.add(self.justifyCenter)
            self.topPanel.add(self.justifyRight)

        if self.extended is not None:
            self.strikethrough = self.createToggleButton(Images.strikeThrough,
                                            "strikethrough")
            self.indent = self.createPushButton(Images.indent,
                                            "indent")
            self.outdent = self.createPushButton(Images.outdent,
                                            "outdent")
            self.hr = self.createPushButton(Images.hr,
                                            "hr")
            self.ol = self.createPushButton(Images.ol,
                                            "ol")
            self.ul = self.createPushButton(Images.ul,
                                            "underline")
            self.insertImage = self.createPushButton(Images.insertImage,
                                            "insert image")
            self.createLink = self.createPushButton(Images.createLink,
                                            "create link")
            self.removeLink = self.createPushButton(Images.removeLink,
                                            "remove link")
            self.removeFormat = self.createPushButton(Images.removeFormat,
                                            "remove formatting")

            self.topPanel.add(self.strikethrough)
            self.topPanel.add(self.indent)
            self.topPanel.add(self.outdent)
            self.topPanel.add(self.hr)
            self.topPanel.add(self.ol)
            self.topPanel.add(self.ul)
            self.topPanel.add(self.insertImage)
            self.topPanel.add(self.createLink)
            self.topPanel.add(self.removeLink)
            self.topPanel.add(self.removeFormat)


        if self.basic is not None:
            self.backColors = self.createColorList("Background")
            self.foreColors = self.createColorList("Foreground")
            self.fonts = self.createFontList()
            self.fontSizes = self.createFontSizes()
            self.bottomPanel.add(self.backColors)
            self.bottomPanel.add(self.foreColors)
            self.bottomPanel.add(self.fonts)
            self.bottomPanel.add(self.fontSizes)

            # We only use these listeners for updating status,
            # so don't hook them up
            # unless at least self.basic editing is supported.
            self.richText.addKeyboardListener(self)
            self.richText.addClickListener(self)



    def createColorList(self, caption):
        lb = ListBox()
        lb.addChangeListener(self)
        lb.setVisibleItemCount(1)

        lb.addItem(caption)
        lb.addItem("White", "white")
        lb.addItem("Black", "black")
        lb.addItem("Red", "red")
        lb.addItem("Green", "green")
        lb.addItem("Yellow", "yellow")
        lb.addItem("Blue", "blue")
        return lb


    def createFontList(self):
        lb = ListBox()
        lb.addChangeListener(self)
        lb.setVisibleItemCount(1)

        lb.addItem("Font", "")
        lb.addItem("Normal", "")
        lb.addItem("Times New Roman", "Times New Roman")
        lb.addItem("Arial", "Arial")
        lb.addItem("Courier New", "Courier New")
        lb.addItem("Georgia", "Georgia")
        lb.addItem("Trebuchet", "Trebuchet")
        lb.addItem("Verdana", "Verdana")
        return lb


    def createFontSizes(self):
        lb = ListBox()
        lb.addChangeListener(self)
        lb.setVisibleItemCount(1)

        lb.addItem("Size")
        lb.addItem("XXsmall")
        lb.addItem("Xsmall")
        lb.addItem("small")
        lb.addItem("medium")
        lb.addItem("large")
        lb.addItem("Xlarge")
        lb.addItem("XXlarge")
        return lb


    def createPushButton(self, img, tip):
        img = Image(img)
        pb = PushButton(img, img, self)
        pb.setTitle(tip)
        return pb


    def createToggleButton(self, img, tip):
        img = Image(img)
        tb = ToggleButton(img, img, self)
        tb.setTitle(tip)
        return tb

    def updateStatus(self):
        """* Updates the status of all the stateful buttons.
        """
        if self.basic is not None:
            self.bold.setDown(self.basic.isBold())
            self.italic.setDown(self.basic.isItalic())
            self.underline.setDown(self.basic.isUnderlined())
            self.subscript.setDown(self.basic.isSubscript())
            self.superscript.setDown(self.basic.isSuperscript())

        if self.extended is not None:
            self.strikethrough.setDown(self.extended.isStrikethrough())

    def onChange(self, sender):
        if sender == self.backColors:
            bc = self.backColors.getValue(self.backColors.getSelectedIndex())
            self.basic.setBackColor(bc)
            self.backColors.setSelectedIndex(0)
        elif sender == self.foreColors:
            fc = self.foreColors.getValue(self.foreColors.getSelectedIndex())
            self.basic.setForeColor(fc)
            self.foreColors.setSelectedIndex(0)
        elif sender == self.fonts:
            fname = self.fonts.getValue(self.fonts.getSelectedIndex())
            self.basic.setFontName(fname)
            self.fonts.setSelectedIndex(0)
        elif sender == self.fontSizes:
            fs = self.fontSizesConstants[self.fontSizes.getSelectedIndex() - 1]
            self.basic.setFontSize(fs)
            self.fontSizes.setSelectedIndex(0)

    def onClick(self, sender):

        if sender == self.bold:
            self.basic.toggleBold()
        elif sender == self.italic:
            self.basic.toggleItalic()
        elif sender == self.underline:
            self.basic.toggleUnderline()
        elif sender == self.subscript:
            self.basic.toggleSubscript()
        elif sender == self.superscript:
            self.basic.toggleSuperscript()
        elif sender == self.strikethrough:
            self.extended.toggleStrikethrough()
        elif sender == self.indent:
            self.extended.rightIndent()
        elif sender == self.outdent:
            self.extended.leftIndent()
        elif sender == self.justifyLeft:
            self.basic.setJustification(RichTextArea.LEFT)
        elif sender == self.justifyCenter:
            self.basic.setJustification(RichTextArea.CENTER)
        elif sender == self.justifyRight:
            self.basic.setJustification(RichTextArea.RIGHT)
        elif sender == self.insertImage:
            url = Window.prompt("Enter an image URL:", "http:#")
            if url is not None:
                self.extended.insertImage(url)

        elif sender == self.createLink:
            url = Window.prompt("Enter a link URL:", "http:#")
            if url is not None:
                self.extended.createLink(url)

        elif sender == self.removeLink:
            self.extended.removeLink()
        elif sender == self.hr:
            self.extended.insertHorizontalRule()
        elif sender == self.ol:
            self.extended.insertOrderedList()
        elif sender == self.ul:
            self.extended.insertUnorderedList()
        elif sender == self.removeFormat:
            self.extended.removeFormat()
        elif sender == self.richText:
            # We use the RichTextArea's onKeyUp event to update the
            # toolbar status.  This will catch any cases where the
            # user moves the cursor using the keyboard, or uses one of
            # the browser's built-in keyboard shortcuts.
            self.updateStatus()

    def onKeyDown(self, sender, keyCode, modifiers):
        pass

    def onKeyPress(self, sender, keyCode, modifiers):
        pass

    def onKeyUp(self, sender, keyCode, modifiers):
        if sender == self.richText:
            # We use the RichTextArea's onKeyUp event to update the
            # toolbar status.  This will catch any cases where the user
            # moves the cursor using the keyboard, or uses one of
            # the browser's built-in keyboard shortcuts.
            self.updateStatus()


