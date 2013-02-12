'''
Created on Jul 31, 2012

@author: meyer
'''
import sys
from annotation.publicInterface import annotationInterface as ani
from PyQt4 import QtCore, QtGui, uic, Qt
from models import *
import operator
import annotation.Sentence
import os
import parsing.Trees
import pickle
#import threading
from knowrobinterface import actions as kr
from annotation.Sentence import WordEntity
from reportlab.lib.validators import isInstanceOf

#Provide all the functions to load data and react to events
class WidgetFunctionality(QtCore.QObject):
    
    def __init__(self,UI):
        QtCore.QObject.__init__(self)
        self.ui = UI
        self.anno = None
        self.currentText = 1
        self.currentSense = None
        self.currentWord = None
        self.currentEntity = None
        self.DepTemplates = []
        self.knowrob = kr.KnowRob('ias_semantic_map',self)
        self.available_knowrob_fields = []
        
    
    def rwchanged(self):
        """Indicate if the real world objects should be part of the mlns."""
        self.anno.create_rw_objects = not self.anno.create_rw_objects
    
    def manualSenseQuery(self):
        """Query for senses with manual user input."""
        
        queryWord = str(self.ui.manualSenseQueryTextEdit.toPlainText())
        queryWord = queryWord.replace(" ", "_")
        
        self.currentWord.manual_word_representations.append(queryWord)
        self.loadSenses(self.currentWord)
        
    def addVirtualWord(self):
        """Add a new virtual word to a parse tree."""
        
        #Check if a blank input was given
        if str(self.ui.newVirtualWordLineEdit.text()) == "":
            msgBox = QtGui.QMessageBox()
            msgBox.setText("Please provide the name of the virtual word.")
            msgBox.exec_()
        virtual_word_name = self.ui.newVirtualWordLineEdit.text()
        #Check if only one syntax tree is selected
        selected_tree_items = self.ui.syntacticTree.selectedItems()
        sentences = []
        for selected_item in selected_tree_items:
            sentence_prop = self.find_sentence_for_node(selected_item)
            sentences.append(sentence_prop[0])
        if not sentences.count(sentences[0]) == len(sentences):
            msgBox = QtGui.QMessageBox()
            msgBox.setText("Please select only one syntax tree.")
            msgBox.exec_()
        #create sub branch virtual words
        virtual_words_exist = False
        i = 0
        while i < sentence_prop[1].childCount():
            child = sentence_prop[1].child(i)
            if child.text(0) == "VIRTUAL":
                virtual_word_node = child
                virtual_words_exist = True
            i+=1
        virtual_tree = None
        if not virtual_words_exist:
            virtual_tree = self.add_virtual_node_to_model(sentence_prop[0])
            virtual_word_node = QtGui.QTreeWidgetItem()
            virtual_word_node.setText(0, "VIRTUAL")
            sentence_prop[1].addChild(virtual_word_node)
        else:
            
            virtual_tree = self.get_virtual_tree_from_syntax_tree(
                                                            sentence_prop[0])
        assert(isinstance(virtual_tree, parsing.Trees.SyntacticTree))
        assert(not virtual_tree.element is None)
        
        #insert virtual word
        virtual_word = QtGui.QTreeWidgetItem()
        virtual_word.setText(0, virtual_word_name)
        virtual_word_node.addChild(virtual_word)
        #update tree represenation
        self.add_virtual_word_to_model(virtual_word_name, virtual_tree)
    
    def get_virtual_tree_from_syntax_tree(self, sentence_id):
        """Get the virtual tree given a sentence id."""
        
        t = self.anno.texts[int(self.currentText)].text.sentences[
                                                            sentence_id].tree
        i = 0
        while i < len(t.children):
            if t.children[i].element.identifier == "virtual_words":
                return t.children[i]
            i += 1
        return None
            
    
    def add_virtual_node_to_model(self, sentence_id):
        """Add the virtual node as child of the root node of a syntax tree."""
        
        t = self.anno.texts[
                        int(self.currentText)].text.sentences[sentence_id].tree
        virtual_tree_element = parsing.Trees.SyntacticElement("virtual_words",
                                                              "virtual_words")
        virtual_tree = parsing.Trees.SyntacticTree()
        virtual_tree.element = virtual_tree_element
        virtual_tree_element.pos = "NN"
        t.addChild(virtual_tree, sentence_id)
        
        return virtual_tree
        
    def add_virtual_word_to_model(self, text, virtual_tree):
        """Add a new virtual word to a sentence."""
            
        virtual_word_tree = parsing.Trees.SyntacticTree()
        virtual_word_element = parsing.Trees.SyntacticElement(str(text), 
                                                              str(text))
        virtual_word_tree.element = virtual_word_element
        
        virtual_tree.addChild(virtual_word_tree, 
                              virtual_tree.sentence)               
        
    
    def find_sentence_for_node(self, selected_item):
        """Find the sentence that an item in the syntax tree belongs to."""
        
        tmp_item = selected_item
        while True:
            if tmp_item.parent() is None:
                break
            tmp_item  = tmp_item.parent()
            
        result = tmp_item.text(0).split('_',1)[1]
        return (str(result), tmp_item)
    
    def deleteDepdendency(self):
        """Delete the selected dependency from the widget and set."""
        
        delList = []
        for dep in self.anno.texts[int(self.currentText)].text.dependencies:
            if str(dep.identifier) == str(
                        self.ui.dependencyListWidget.selectedItems()[0].text()):
                self.ui.dependencyListWidget.takeItem(
                        self.ui.dependencyListWidget.row(
                        self.ui.dependencyListWidget.selectedItems()[0]))
                delList.append(dep)
                break
        if len(delList) >0:
            for dep in delList:
                self.anno.texts[
                            int(self.currentText)].text.dependencies.remove(dep)
    
    def addKRFields(self):
        add_items_dialog = knowrobItemsDialog(self.available_knowrob_fields, 
                                              self.knowrob)
        if add_items_dialog.exec_():
            vals = add_items_dialog.getValue()
        self.available_knowrob_fields = vals
        self.ui.knowRobListWidget.clear()
        for val in vals:
            self.ui.knowRobListWidget.addItem("'{item}'".format(item=val))
        
    def loadGroundings(self):
        """Load the groundings for all phrases into the UI."""
        
        self.ui.groundedListWidget.clear()
        if len(self.anno.texts[int(self.currentText)].text.sentences) == 0:
            return None
        for sentence in self.anno.texts[
                            int(self.currentText)].text.sentences.itervalues():
            
            for phrase in sentence.tree.inOrderLeafs():
                if phrase.element.groundIdentifier is None:
                    continue
                addString = "{phraseString} -> {element}".format(
                                phraseString = phrase.element.mlnidentifier, 
                                element = phrase.element.groundIdentifier)
                self.ui.groundedListWidget.addItem(addString)  
            
    
    def addGrounding(self):
        selecteditem = self._matchUiToDataModelTree(
                                        self.ui.syntacticTree.selectedItems())
        if not len(selecteditem) == 1 or selecteditem[0] is None:
            msgBox = QtGui.QMessageBox()
            msgBox.setText("Please select exactly one element in the sentence!")
            msgBox.exec_()
            return
        selecteditem[0].element.groundIdentifier = str(
                            self.ui.knowRobListWidget.selectedItems()[0].text())

        addString = "{phrase} -> {element}".format(
                phrase = selecteditem[0].element.mlnidentifier, 
                element = self.ui.knowRobListWidget.selectedItems()[
                                                    0].text().split('#',1)[1])
        self.ui.groundedListWidget.addItem(addString)  
    
    def deleteGrounding(self):
        """Delete the grouding for one phrase and update UI."""
        
        selectedItem = self.ui.groundedListWidget.currentItem().text()
        currentPhrase = selectedItem.split(' ->')[0]
        word, sentence = self.getWordAndSentenceIdentifierFromMLNIdentifier(
                                                                currentPhrase)
        s = self.anno.texts[int(self.currentText)].text.sentences[sentence]
        s.tree.getSubTreeByIdentifier(word).element.groundIdentifier = None
        self.ui.groundedListWidget.takeItem(
                                        self.ui.groundedListWidget.currentRow())
        
    def saveProgressToSQLite(self):
        """Save progress to SQLite Database."""

        saveTo = QtGui.QFileDialog.getSaveFileName(self.ui, 'save to .db file')
        self.anno.saveTextsToSQLite(str(saveTo))
        
    def getWordAndSentenceIdentifierFromMLNIdentifier(self,mlnidentifier):
        phraseParts = mlnidentifier.split('_')
        currentWord = '{p1}-{p2}'.format(p1 = phraseParts[0], 
                                         p2 = phraseParts[1])
        currentSentence = '{p}'.format(p = phraseParts[3])
        return [currentWord, currentSentence]
    
    def deleteKRField(self):
        self.ui.knowRobListWidget.takeItem(
                                        self.ui.knowRobListWidget.currentRow())
    
    def updateKnowRobList(self, clickedItem):
        """Add the clicked item into the KnowRob List."""
        
        self.available_knowrob_fields.append(str(clickedItem).strip("'"))
        self.ui.knowRobListWidget.addItem(clickedItem)
    
    def loadDependencyTemplates(self):
        files = QtGui.QFileDialog.getOpenFileNames(self.ui, 
                                                   'open file *.depTemp')
        for f in files:
            self.loadDependencyTemplateFile(f)
            
    def loadProgressFromFile(self, filename):
        try:
            f =open(filename,'r')
            self.texts = pickle.load(f)
        except Exception:
            return False
        return True
    
    def saveDependencyTemplates(self):
        saveTo = QtGui.QFileDialog.getSaveFileName(self.ui, 
                                                   'Save to *.depTemp file')
        try:
            f = open(saveTo,'w')
            pickle.dump(self.DepTemplates, f, pickle.HIGHEST_PROTOCOL)
            f.close()
        except:
            print "Error saving to file."
    
    def loadDependencyTemplateFile(self, filename):
        #try:
        f = open(filename,'r')
        self.DepTemplates = pickle.load(f)
        self.loadDepTemplates()
        f.close()
        #except Exception:
        #    return False
        #return True
    
    def loadDepTemplates(self):
        for t in self.DepTemplates:
            insert = True
            for i in self.ui.depTemplateList.findItems(QtCore.QString("*"), 
                                                       QtCore.Qt.MatchWildcard):
                if i.text() == t.name:
                    insert=False
            if insert:
                self.ui.depTemplateList.addItem(t.name)
            
    def chunkChanges(self):

        selection = self.ui.syntacticTree.selectedItems() 
        self.anno.texts[int(self.currentText)].text.sentences
        
    def findAllLeafs(self, tree):
        res = []
        if len(tree.children) == 0:
            return [tree.text(0)]
        else:
            for child in tree.child:
                return res + self.findAllLeafs(tree) 
        return ''
    
    def previewMLN(self):
        mln = self.anno.getMLNbyTextID(self.currentText.toInt()[0])
        mlnpreview = mlnPreview(mln, self.anno, self.currentText)
        if mlnpreview.exec_():
            pass

    def highlightWords(self, words):
        """Highlight all the words that are provided in the parameter."""
        
        sentences = sorted(self.anno.texts[
                            int(self.currentText)].text.sentences.iteritems(), 
                           key = operator.itemgetter(0))
        row = 0
        greenBrush = QtGui.QBrush(QtGui.QColor(161, 255, 175))
        plainBrush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        for sentence in sentences:
            col = 0
            sentenceWords = sentence[1].words
            for word in sentenceWords.values():
                for w in words:
                    if w == word:
                        self.ui.sentenceTableWidget.item(row,
                                                         col).setBackground(
                                                                    greenBrush)
                        break
                    else:
                        if self.ui.sentenceTableWidget.item(row,
                                                            col) is not None:
                            self.ui.sentenceTableWidget.item(row,
                                                             col).setBackground(
                                                                    plainBrush)
                col += 1    
            row += 1
                        
    def loadNewFiles(self):
        files = QtGui.QFileDialog.getOpenFileNames(self.ui, 'open file')
        if len(files) == 0:
            return
        if self.anno is None:
            self.anno = ani(texts = None, fileList = files, kr = self.knowrob)
        else:
            self.anno.addFiles(files)
        self.populateWidgets(self.anno.texts)
    
    def loadQueryFile(self):
        files = QtGui.QFileDialog.getOpenFileNames(self.ui, 
                                                   'open file to query')
        if files is None:
            return None
        queryAnno = ani(texts = None, fileList = files)
        queryDia = queryDialog(queryAnno.getMLN())
        if queryDia.exec_():
            val = queryDia.getValues()
        
    def loadFileList(self):
        file = None
        file = QtGui.QFileDialog.getOpenFileName(parent=self.ui, 
                                                 caption=QtCore.QString(
                                                                'Select list'))
        if len(file) == 0:
            return
        if not file is None:
            files = []
            for f in open(file).readlines():
                files.append(f.rstrip('\n'))
            if self.anno is None:
                self.anno = ani(texts= None,fileList = files)
            else:
                self.anno.addFiles(files)
            self.populateWidgets(self.anno.texts)
            
    def saveToFile(self):
        saveTo = QtGui.QFileDialog.getSaveFileName(self.ui, 'save to pkl file')
        if self.anno is not None:
            self.anno.saveData(saveTo)
    
    def openProgressfromFile(self):
        filename = QtGui.QFileDialog.getOpenFileName(self.ui, 
                                                     'open progress file')
        if self.anno is None:
            self.anno = ani(None,None)
        self.anno.loadProgressFromFile(filename)
        self.populateWidgets(self.anno.texts)
    
    def closeApp(self):
        QtGui.QApplication.exit(1)
        
    def exportToMLN(self):
        folder = QtGui.QFileDialog.getExistingDirectory(self.ui, 
                                                    'Select an output folder')
        self.anno.saveToMLNDatabase(folder)
    
    def exportSelectedtoMLN(self):
        folder = QtGui.QFileDialog.getExistingDirectory(self.ui, 
                                                    'Select an output folder')
        self.anno.saveSelectedtoMLN(folder, self.currentText)
    
    def textChanged(self, selected, deselected):
        self.currentText = selected[0].indexes()[0].data().toString()
        self.saveChanges(deselected)
        self.loadTextDetails(self.currentText)
        self.loadGroundings()
        self.loadEntities()
        self.loadDependencyEntities()

        #self.loadDependencies(selected[0].indexes()[0].data().toString())
    
    def wordChanged(self, row, column):
        self.loadDependencies(row, column)
        self.loadWordDetails(row,column)
        #self.highlightEntity(row,column)
        
    def loadWordDetails(self,row,column):
        words = sorted(self.anno.texts[
                            int(self.currentText)].text.sentences.iteritems(), 
                       key=operator.itemgetter(0))[row][1].words
        w = words[column]
        #w = self.anno.texts[int(self.currentText)].text.sentences.values()[row].words[sorted(sorted(self.anno.texts[int(self.currentText)].text.sentences.values())[row].words.keys())[column]]
        self.currentWord = w
        self.loadSenses(w)
        
    def setTableBackGroundColor(self, color):
        sentences = sorted(self.anno.texts[
                            int(self.currentText)].text.sentences.iteritems(), 
                           key = operator.itemgetter(0))
        row = 0
        for sentence in sentences:
            col = 0
            sentenceWords = sentence[1].words
            for word in sentenceWords.values():
                if self.ui.sentenceTableWidget.item(row,col) is not None:
                    self.ui.sentenceTableWidget.item(row,
                                                     col).setBackground(color)
                col += 1    
            row += 1
    
    def loadPath(self):
        self.ui.pathListWidget.clear()
        path = self.currentSense.sensePathNLTK
        pathSet = set()
        for p in path:
            for s in p:
                pathSet.add(s)
        for s in pathSet:
            self.ui.pathListWidget.addItem(s.name)
    
    def loadSenses(self, word):
        self.ui.senseListWidget.clear()
        self.ui.pathListWidget.clear()
        self.ui.senseDescriptionTextEdit.clear()
        self.ui.exampleListWidget.clear()
        possibleSenses = word.possibleSenses
        #senses = word._sense
        
        for s in possibleSenses:
            self.ui.senseListWidget.addItem(s._senseID)
            if s._senseID == word.majorityRawSense:
                self.currentSense = s
                self.ui.senseListWidget.item(
                            self.ui.senseListWidget.count()-1).setSelected(True)

    def senseClicked(self, item):
        """
        Check if the selection has changed or if the selection needs to be
        deactivated.
        
        """
        if self.currentSense is not None:
            if self.currentSense._senseID == str(item.text()) and ( 
                                                self.currentSense is not None):
                item.setSelected(False)
                self.currentSense=None
                self.ui.senseDescriptionTextEdit.clear()
                self.ui.exampleListWidget.clear()
                self.ui.pathListWidget.clear()
                self.currentWord.setSense(None)
                return True
        return False
    
    def senseChanges(self, item):
        """Update the description, examples and the path."""
        
        self.ui.senseDescriptionTextEdit.clear()
        cs = self.ui.senseListWidget.selectedItems()
        if(self.senseClicked(cs[0])):
            return
        #Determine the currently selected sense
        if len(cs)>0:   
            for ps in self.currentWord.possibleSenses:
                if cs[0].text() == ps._senseID:
                    self.currentSense = ps
                    self.currentWord.setSense(cs[0].text())
        
        self.ui.senseDescriptionTextEdit.insertPlainText(
                                                self.currentSense.definition)
        
        self.ui.exampleListWidget.clear()
        if self.currentSense.examples is not None:
            for example in self.currentSense.examples:
                self.ui.exampleListWidget.addItem(example)
                
        self.loadPath()
    
    def sentenceChanged(self, selected, deselected):
        pass
    
    def loadEntities(self):
        """Load the entities and display them in the GUI"""
        #self.ui.chunkListWidget.clear()
        self.ui.syntacticTree.clear()
        self.ui.syntacticTree.setColumnCount(1)
        toplevelitems = []
        for key, value in self.anno.texts[
                            int(self.currentText)].text.sentences.iteritems():
            toplevelitems.append(QtGui.QTreeWidgetItem(['S_' + str(key)],0))
            toplevelitems[len(toplevelitems)-1].addChild(
                                                    value.tree.getQtElement())

        self.ui.syntacticTree.insertTopLevelItems(0,toplevelitems)

    def getSyntacticElementTree(self, synel, sentenceID):
        res = None
        
        if isinstance(synel, parsing.Trees.SyntacticTree):
            res = QtGui.QTreeWidgetItem([synel.element.identifier],0)
        else:
            res = QtGui.QTreeWidgetItem([str(synel.identifier) + 
                                         '_S_' + str(sentenceID)],0)
        if isinstance(synel, annotation.Sentence.WordEntity):
            return res
        elif isinstance(synel, annotation.Sentence.SentenceChunk):
            for idx, w in enumerate(synel.words):
                res.addChild(self.getSyntacticElementTree(w, sentenceID))
        return res

    
    def loadDependencies(self, row, column):
        """Load the dependencies for one sentence into the table"""
        i=0
        wordDeps = self.anno.texts[
                                int(self.currentText)].text.sentences.values()[
                                                            row].dependencies
        self.ui.dependencyTableWidget.setRowCount(len(wordDeps))
        self.ui.dependencyTableWidget.setColumnCount(3)
        
        for dep in wordDeps:
            self.ui.dependencyTableWidget.setItem(i, 
                                                0, 
                                                QtGui.QTableWidgetItem(dep.typ))
            self.ui.dependencyTableWidget.setItem(i, 
                                        1, 
                                        QtGui.QTableWidgetItem(dep.first.word))
            self.ui.dependencyTableWidget.item(i,
                                        1).setFlags(QtCore.Qt.ItemIsSelectable)
            self.ui.dependencyTableWidget.setItem(i, 
                                        2, 
                                        QtGui.QTableWidgetItem(dep.second.word))
            self.ui.dependencyTableWidget.item(i,
                                        2).setFlags(QtCore.Qt.ItemIsSelectable)
            i+=1
    
    def depChanges(self, row, column):
        #TODO: Implement
        pass
    
    def loadDependencyEntities(self):
        self.ui.dependencyListWidget.clear()
        for dep in self.anno.texts[int(self.currentText)].text.dependencies:
            self.ui.dependencyListWidget.addItem(dep.identifier)
    
    def loadTextDetails(self, index):
        self.ui.sentenceTableWidget.clear()
        i=0
        maxcols = 0
        self.ui.sentenceTableWidget.setRowCount(len(self.anno.texts[
                                            int(index)].text.sentences.keys()))
        for sentenceID in sorted(self.anno.texts[
                                        int(index)].text.sentences.iterkeys()):
            sentence = self.anno.texts[int(index)].text.sentences[sentenceID]
            for id in sentence.words.itervalues():
                if id.index > maxcols:
                    maxcols = id.index
            self.ui.sentenceTableWidget.setColumnCount(maxcols+1)
            for wordID in sorted(sentence.words.iterkeys()):
                self.ui.sentenceTableWidget.setItem(i, 
                                                    wordID, 
                                                    QtGui.QTableWidgetItem(
                                                                sentence.words[
                                                                wordID].word))
            i+=1
                
    def saveChanges(self, deselected):
        """Save all changes made to one text"""
        pass
        
    def populateWidgets(self, texts):
        self.ui.textListWidget.clear()
        for key in self.anno.texts.iterkeys():
            self.ui.textListWidget.addItem(str(key))
            
    def mergeButtonClicked(self):
        """
        Create a new entity that consists of the words 
        that are selected in the table.
        
        """
        
        selectedCells = self.ui.sentenceTableWidget.selectedItems()
        #Create a new entity that is blank

        
        for selectedCell in selectedCells:
            row = selectedCell.row()
            col = selectedCell.column()
            sentenceWords = sorted(self.anno.texts[
                            int(self.currentText)].text.sentences.iteritems(), 
                            key=operator.itemgetter(0))[row][1].words
            word = sentenceWords[col]
            if word.syntacticObject is not None:
                msgBox = QtGui.QMessageBox()
                msgBox.setText("The word '%s' is already part of entity: '%s'"%(word.word,word.syntacticObject.identifier))
                msgBox.exec_()
                return
            
        self.anno.texts[int(self.currentText)].text.entityDict['ent' 
            + str(len(self.anno.texts[
            int(self.currentText)].text.entityDict.keys()))
                            ] = annotation.Sentence.SentenceChunk('ent' 
                                + str(len(self.anno.texts[
                                int(self.currentText)].text.entityDict.keys())))
        for selectedCell in selectedCells:
            row = selectedCell.row()
            col = selectedCell.column()
            sentenceWords = sorted(self.anno.texts[
                            int(self.currentText)].text.sentences.iteritems(), 
                            key=operator.itemgetter(0))[row][1].words
            word = sentenceWords[col]
            if word.syntacticObject is None:
                self.anno.texts[int(self.currentText)].text.entityDict['ent' 
                    + str(len(self.anno.texts[
                        int(self.currentText)].text.entityDict.keys())-1)
                                                            ].words.add(word)
                word.syntacticObject = self.anno.texts[
                                    int(self.currentText)].text.entityDict['ent' 
                                + str(len(self.anno.texts[int(self.currentText)
                                                ].text.entityDict.keys())-1)]

            
        self.loadEntities()

    def newDepTemp(self):
        dia = tempDialog()
        val = None
        if dia.exec_():
            val = dia.getValues()
        if val is not None:
            self.DepTemplates.append(val)
            self.ui.depTemplateList.addItem(val.name)
                
    def removeButtonClicked(self):
        """Remove entity currently selected in the enityListWisget"""
        #self.ui.entityWordListWidget.clear()
        entity = self.ui.chunkListWidget.selectedItems()[0]
        entityIdentifier = entity.text()
        
        item = self.ui.chunkListWidget.takeItem(
                                        self.ui.chunkListWidget.currentRow())
        item = None
        
        for word in self.anno.texts[int(self.currentText)].text.entityDict[
                                                str(entityIdentifier)].words:
            word.syntacticObject = None
            
        del self.anno.texts[int(self.currentText)].text.entityDict[
                                                        str(entityIdentifier)]
        self.ui.chunkListWidget.setCurrentItem(None)
        self.setTableBackGroundColor(QtGui.QBrush(QtGui.QColor(255,255,255)))
    
    def _matchUiToDataModelTree(self, items):
        res = []
        for i in items:
            root = self._findRootNodeOfTreeWidgetItem(i)
            sentenceID = str(root.text(0)).rsplit('_',1)[1]
            sentence = self.anno.texts[int(self.currentText)].text.sentences[
                                                                    sentenceID]
            res.append(sentence.tree.getSubTreeByIdentifier(i.text(0)))
            
        return res
    
    def _findRootNodeOfTreeWidgetItem(self,item):
        if item.parent() is None:
            return item
        else:
            return self._findRootNodeOfTreeWidgetItem(item.parent())
    
    def newDependency(self):

        selectedTemplateRow = self.ui.depTemplateList.currentRow()
        selectedTemplate = self.DepTemplates[selectedTemplateRow]
        entities = self._matchUiToDataModelTree(
                                        self.ui.syntacticTree.selectedItems())
        
        if len(entities) == 0:
            return
        if not selectedTemplate.entities == len(
                                        self.ui.syntacticTree.selectedItems()):
            msgBox = QtGui.QMessageBox()
            msgBox.setText("""could note create the dependency!
                        (Maybe not enough words?Wrong number of Entities?)""")
            msgBox.exec_()
            return
        
        newDep = newDependency(selectedTemplate, entities)
        
        if newDep.exec_():
            val = newDep.getValue()
        #get all the words
        if not newDep is False:
            self.anno.texts[int(self.currentText)].text.dependencies.add(val)
            self.ui.dependencyListWidget.addItem(val.identifier)
      
    def qListItemRow(self, qlist, item):
        j = 0
        while not j == qlist.count():
            if item == qlist.item(j):
                return j
            j+=1
        return False

class wordOrEntityDialog(QtGui.QDialog):
    def __init__(self, word):
        QtGui.QDialog.__init__(self)
        self.ui = uic.loadUi('ui/wordOrEntityDialog.ui', self)
        self.ui.WordLabel.setText(word)
    
    def getValues(self):
        if self.ui.wordRadio.isChecked():
            return 'w'
        return 'e'

class tempDialog(QtGui.QDialog):
    def __init__(self):
        QtGui.QDialog.__init__(self)
        self.ui = uic.loadUi('ui/newTemplateDialog.ui', self)
        self.setUpSignals()
    
    def setUpSignals(self):
        self.ui.addTempButton.clicked.connect(self.addTemp)
        self.ui.removeTemplateButton.clicked.connect(self.removeTemp)
        
    def removeTemp(self):
        delItem = self.ui.outputList.takeItem(self.ui.outputList.currentRow())
        delItem = None
    
    def addTemp(self):
        self.ui.outputList.addItem(self.ui.tmpTemplate.text())
    
    def getValues(self):
        name = str(self.ui.depName.text())
        entities = int(self.ui.entityNumber.text())
        templates = []
        i =0
        if not self.ui.outputList.count() == 0:
            while self.ui.outputList.item(i):
                templates.append(str(self.ui.outputList.item(i).text()))
                i+=1
        return dependencyTemplate(name, entities, templates)

class queryDialog(QtGui.QDialog):
    def __init__(self, queryMLN):
        QtGui.QDialog.__init__(self)
        self.ui = uic.loadUi('ui/queryOutputDialog.ui', self)
        self.setUpWidget(queryMLN)
        self.setUpSignals()
    
    def setUpWidget(self, queryMLN):
        self.ui.queryMLNTextWidget.setText(queryMLN)
    
    def setUpSignals(self):
        pass
        
    def getValues(self):
        return None
    
class mlnPreview(QtGui.QDialog):
    def __init__(self, mln, anno, current):
        QtGui.QDialog.__init__(self)
        self.ui = uic.loadUi('ui/mlnPreview.ui', self)
        self.setUpSignals()
        self.setUpWidget(mln)
        self.mln = mln
        self.anno = anno
        self.currentText = current
    
    def setUpSignals(self):
        self.ui.savePushButton.clicked.connect(self.saveMLNToFile)
    
    def saveMLNToFile(self):
        saveTo = QtGui.QFileDialog.getSaveFileName(self.ui, 'save to .db file')
        self.anno.saveSelectedtoMLN('', self.currentText, saveTo)
    
    def setUpWidget(self, mln):
        self.ui.mlnPlainTextWidget.setPlainText(mln)

class knowrobItemsDialog(QtGui.QDialog):
    def __init__(self, already_added_items, knowrob):
        QtGui.QDialog.__init__(self)
        self.knowrob = knowrob
        self.ui = uic.loadUi('ui/addKRItems.ui', self)
        self.current_items = already_added_items
        spatial_objects_query = """owl_individual_of(A, 
                                    knowrob:'EnduringThing-Localized')"""
        
        self.available_knowrob_fields = set([x['A'] for x in 
                                             self.knowrob.knowrobQuery(
                                                        spatial_objects_query)])
        
        for f in self.available_knowrob_fields:
            self.ui.kritems.addItem(f)
            if f in self.current_items:
                self.ui.kritems.item(
                                    self.ui.kritems.count()-1).setSelected(True)
                
    def getValue(self):
        qres = self.ui.kritems.selectedItems()
        res = [str(x.text()) for x in qres]
        return res
        

class newDependency(QtGui.QDialog):
    def __init__(self, template, entities):
        QtGui.QDialog.__init__(self)
        self.ui = uic.loadUi('ui/depDialog.ui', self)
        self.setUpWidget(entities, template)
        self.setUpSignals()
        self.outputTemplate = template
        self.entities = entities

    def setUpSignals(self):
        self.ui.upButton.clicked.connect(self.moveWordUp)
        self.ui.downButton.clicked.connect(self.moveWordDown)
        self.ui.deleteButton.clicked.connect(self.deleteEntityFromList)
        
    def addWordToList(self):
        self.ui.wordList.addItem(self.ui.word.text())
        self.words[self.ui.word.text()] = str(self.ui.word.text())
    
    def deleteEntityFromList(self):
        item = self.ui.wordListWidget.takeItem(
                                            self.ui.wordListWidget.currentRow())
        #self.entities[item.text()] = None
    
    def moveWordUp(self):
        idx = self.ui.wordListWidget.currentRow()
        if idx==0:
            return
        item = self.ui.wordListWidget.takeItem(
                                            self.ui.wordListWidget.currentRow())
        self.ui.wordListWidget.insertItem(idx-1, item)
        self.ui.wordListWidget.setCurrentItem(item)
    
    def moveWordDown(self):
        idx = self.ui.wordListWidget.currentRow()
        if idx == self.ui.wordListWidget.count()-1:
            return
        item = self.ui.wordListWidget.takeItem(
                                            self.ui.wordListWidget.currentRow())
        self.ui.wordListWidget.insertItem(idx+1, item)
        self.ui.wordListWidget.setCurrentItem(item)
        
    def getValue(self):
        all_items = self.ui.wordListWidget.findItems('.*', 
                                                     QtCore.Qt.MatchRegExp)
        dep = annotation.Sentence.DepedencyEntity(
                                        self.outputTemplate.name, 
                                            [str(x.text()) for x in all_items], 
                                            self.outputTemplate.templates)
        return dep
    
    def setUpWidget(self, entities, template):
        self.ui.templateNameLabel.setText(template.name)
        for ele in entities:
            self.ui.wordListWidget.addItem(ele.element.mlnidentifier)    

class dependencyTemplate:
    def __init__(self,name,entities,templates):
        self.name = name
        #number of entities required
        self.entities = entities
        #output template for the MLN representation
        self.templates = templates
        
    def createDependencyFromTemplate(self, words, entities):
        """
        Create a dependency with the words and entities provided 
        for this specific template(chosen by user)
        
        """
        if len(entities) != self.entities:
            return False
        return annotation.Sentence.DepedencyEntity(self.name, 
                                                   entities, 
                                                   self.templates)
    
    @property
    def mlnrepresentation(self):
        res = '{template}(...)'.format(
                                    template=self.templates[0].split('(',1)[0])
        return res
        
class WidgetManager(QtGui.QMainWindow):
    
    def __init__(self):
         QtGui.QMainWindow.__init__(self)

         # Set up the user interface from Designer.
         self.ui = uic.loadUi("ui/annotationTool.ui", self)
         
         self.wf = WidgetFunctionality(self)
         self.setUpSignalSlots()
         
         self.loadStandartConfig()
         #Set up all the events in the ui
         #cm = ConnectionManager(self, self.ui)
    
         
         #    
         
    def loadStandartConfig(self):
        this_dir, this_filename = os.path.split(__file__)
        filename = os.path.join(this_dir, 'standartTemplates.depTemp')
        self.wf.loadDependencyTemplateFile(filename)
        
        
    def setUpSignalSlots(self):
        """Initialize all signals and slots"""
        
        self.ui.actionLoad_texts.triggered.connect(self.wf.loadNewFiles)
        self.ui.textListWidget.selectionModel().selectionChanged.connect(
                                                            self.wf.textChanged)
        self.ui.sentenceTableWidget.cellClicked.connect(self.wf.wordChanged)
        self.ui.dependencyTableWidget.cellClicked.connect(self.wf.depChanges)
        self.ui.senseListWidget.itemClicked.connect(self.wf.senseChanges)
        self.ui.actionSave.triggered.connect(self.wf.saveToFile)
        self.ui.actionOpen_progres_file.triggered.connect(
                                                self.wf.openProgressfromFile)
        self.ui.actionExit.triggered.connect(self.wf.closeApp)
        self.ui.actionExport_to_MLN_database.triggered.connect(
                                                            self.wf.exportToMLN)
        self.ui.actionImport_texts_from_list.triggered.connect(
                                                        self.wf.loadFileList)
        self.ui.newTemplateButton.clicked.connect(self.wf.newDepTemp)
        self.ui.newDependencyButton.clicked.connect(self.wf.newDependency)
        self.ui.actionLoad_file_for_Co_Reference_query.triggered.connect(
                                                        self.wf.loadQueryFile)
        self.ui.syntacticTree.itemSelectionChanged.connect(self.wf.chunkChanges)
        self.ui.actionExport_selected_to_MLN.triggered.connect(
                                                    self.wf.exportSelectedtoMLN)
        self.ui.actionMLN_Preview.triggered.connect(self.wf.previewMLN)
        self.ui.actionLoad_dependency_templates.triggered.connect(
                                                self.wf.loadDependencyTemplates)
        self.ui.actionSave_dependency_templates.triggered.connect(
                                                self.wf.saveDependencyTemplates)
        self.ui.deleteKnowRobField.clicked.connect(self.wf.deleteKRField)
        self.ui.groundPushButton.clicked.connect(self.wf.addGrounding)
        self.ui.deleteGroundingPushButton.clicked.connect(
                                                        self.wf.deleteGrounding)
        self.ui.actionSave_to_SQlite.triggered.connect(
                                                self.wf.saveProgressToSQLite)
        self.ui.addKRFieldsButton.clicked.connect(self.wf.addKRFields)
        self.ui.DeleteDependencyButton.clicked.connect(
                                                    self.wf.deleteDepdendency)
        self.ui.manualSenseQueryButton.clicked.connect(self.wf.manualSenseQuery)
        self.ui.newVirtualWordButton.clicked.connect(self.wf.addVirtualWord)
        #Thread communication
        self.connect(self.wf.knowrob, 
                                    QtCore.SIGNAL('updateKnowRobList(QString)'), 
                                    self.wf.updateKnowRobList)
        self.ui.createRealWorldCB.stateChanged.connect(self.wf.rwchanged)
        self.wf.knowrob.start()
                
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = WidgetManager()
    window.show()
    sys.exit(app.exec_())