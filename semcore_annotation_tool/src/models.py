'''
Created on Aug 1, 2012

@author: meyer
'''
from PyQt4 import QtCore, QtGui, uic

class TextListModel(QtCore.QAbstractListModel):
    def __init__(self, datain, parent=None, *args):
        QtCore.QAbstractListModel.__init__(self, parent, *args) 
        self.listdata = datain
 
    def rowCount(self, parent=QtCore.QModelIndex()): 
        return len(self.listdata) 
 
    def data(self, index, role):
        if index.isValid() and role == QtCore.Qt.DisplayRole:
            return QtCore.QVariant(self.listdata[index.row()].text.sentences[self.listdata[index.row()].text.sentences.keys()[0]].sentence)
        else:
            return QtCore.QVariant()
    
    def setData(self, index, value, role = QtCore.Qt.EditRole):
        if role == QtCore.Qt.EditRole:
                row = index.row()
                data = value
                self.listdata.append(value)
                self.dataChanged.emit(index,index)
                return True
        return False
    
    def appendRow(self, value, parent):
        self.beginInsertRows(QtCore.QModelIndex(), len(self.listdata), len(self.listdata))
        self.listdata.append(value)
        self.endInsertRows()
    
    def replaceData(self, newData):
        self.beginResetModel()
        self.listdata = newData
        self.endResetModel()
    
class SentenceListModel(QtCore.QAbstractTableModel):
    def __init__(self, datain, parent=None, *args):
        QtCore.QAbstractTableModel.__init__(self, parent, *args)
        self.tableData = datain
    
    def rowCount(self, parent=QtCore.QModelIndex):
        return len(self.tableData.sentences)
    
    def columnCount(self, parent=QtCore.QModelIndex):
        maxCount = 0
        for s in self.tableData.sentences:
            count = len(s.words)
            if maxCount < count:
                maxCount = count
        return maxCount
    
    def flags(self, index):
        return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable
    
    def data(self, index, role):
        if role == QtCore.Qt.DisplayRole:       
            row = index.row()
            column = index.column()
            value = self.tableData.sentences[row].words[column]
            
        return value
        
        
    def replaceData(self, newData):
        self.beginResetModel()
        self.listdata = newData
        self.endResetModel()