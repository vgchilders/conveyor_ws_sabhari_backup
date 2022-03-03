from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class CreateAnnotationPopUp(QDialog):
    def __init__(self, x, y, global_pos):
        super().__init__()
        self.classes = ['Cardboard', 'Metal', 'Rigid Plastic', 'Soft Plastic']
        self.label = 0
        self.width = 220
        self.height = 150
        self.left = global_pos[0]
        self.top = global_pos[1] - (self.height/2)
        if(global_pos[2] == -1):
            self.left -= self.width

        QDialog.__init__(self)
        self.setWindowTitle("Create")
        self.setGeometry(self.left, self.top, self.width, self.height)
        verticalLayout = QVBoxLayout(self)
        self.text = QLabel()
        self.text.setText(
            'Select type of recycling')
        verticalLayout.addWidget(self.text)

        # Create radio buttons for each class
        self.c0 = QRadioButton(self.classes[0])
        self.c1 = QRadioButton(self.classes[1])
        self.c2 = QRadioButton(self.classes[2])
        self.c3 = QRadioButton(self.classes[3])
        button_list = [self.c0, self.c1, self.c2, self.c3]
        button_list[0].setChecked(True)

        verticalLayout.addWidget(self.c0)
        verticalLayout.addWidget(self.c1)
        verticalLayout.addWidget(self.c2)
        verticalLayout.addWidget(self.c3)
        self.c0.toggled.connect(lambda: self.btnstate(self.c0))
        self.c1.toggled.connect(lambda: self.btnstate(self.c1))
        self.c2.toggled.connect(lambda: self.btnstate(self.c2))
        self.c3.toggled.connect(lambda: self.btnstate(self.c3))
        self.c0_shortcut = QShortcut(QKeySequence('1'), self)
        self.c0_shortcut.activated.connect(self.shortcutc0)
        self.c1_shortcut = QShortcut(QKeySequence('2'), self)
        self.c1_shortcut.activated.connect(self.shortcutc1)
        self.c2_shortcut = QShortcut(QKeySequence('3'), self)
        self.c2_shortcut.activated.connect(self.shortcutc2)
        self.c3_shortcut = QShortcut(QKeySequence('4'), self)
        self.c3_shortcut.activated.connect(self.shortcutc3)

        buttonBox = QDialogButtonBox(self)
        buttonBox.setStandardButtons(
            QDialogButtonBox.Cancel | QDialogButtonBox.Save)
        verticalLayout.addWidget(buttonBox)
        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)
        self.initCreateAnnot()

    def initCreateAnnot(self):
        if self.exec():
            print("Created trash_item type: ", self.label)
        else:
            self.label = -1

    def btnstate(self, b):
        if b.isChecked() == True:
            self.label = self.classes.index(b.text())

    def shortcutc0(self):
        self.c0.click()
        self.accept()

    def shortcutc1(self):
        self.c1.click()
        self.accept()

    def shortcutc2(self):
        self.c2.click()
        self.accept()

    def shortcutc3(self):
        self.c3.click()
        self.accept()


class EditAnnotationPopUp(QDialog):
    def __init__(self, trash_item, trash_items, global_pos):
        super().__init__()
        self.classes = ['Cardboard', 'Metal', 'Rigid Plastic', 'Soft Plastic']
        self.trash_item = trash_item
        self.trash_items = trash_items
        self.updated_conf = self.trash_item.conf
        self.updated_label = self.trash_item.trash_type
        self.delete = False
        self.width = 175
        self.height = 200
        self.left = global_pos[0]
        self.top = global_pos[1] - (self.height/2)
        if(global_pos[2] == -1):
            self.left -= self.width

        QDialog.__init__(self)
        self.setWindowTitle("Edit")
        self.setGeometry(self.left, self.top, self.width, self.height)
        verticalLayout = QVBoxLayout(self)

        # Create radio buttons for each class
        self.text = QLabel()
        self.text.setText(
            'Select type of recycling:')
        verticalLayout.addWidget(self.text)
        self.c0 = QRadioButton(self.classes[0])
        self.c1 = QRadioButton(self.classes[1])
        self.c2 = QRadioButton(self.classes[2])
        self.c3 = QRadioButton(self.classes[3])
        button_list = [self.c0, self.c1, self.c2, self.c3]
        button_list[self.trash_item.trash_type].setChecked(True)
        verticalLayout.addWidget(self.c0)
        verticalLayout.addWidget(self.c1)
        verticalLayout.addWidget(self.c2)
        verticalLayout.addWidget(self.c3)
        self.c0.toggled.connect(lambda: self.btnstate(self.c0))
        self.c1.toggled.connect(lambda: self.btnstate(self.c1))
        self.c2.toggled.connect(lambda: self.btnstate(self.c2))
        self.c3.toggled.connect(lambda: self.btnstate(self.c3))
        self.c0_shortcut = QShortcut(QKeySequence('1'), self)
        self.c0_shortcut.activated.connect(self.shortcutc0)
        self.c1_shortcut = QShortcut(QKeySequence('2'), self)
        self.c1_shortcut.activated.connect(self.shortcutc1)
        self.c2_shortcut = QShortcut(QKeySequence('3'), self)
        self.c2_shortcut.activated.connect(self.shortcutc2)
        self.c3_shortcut = QShortcut(QKeySequence('4'), self)
        self.c3_shortcut.activated.connect(self.shortcutc3)


        buttonBox = QDialogButtonBox(self)
        buttonBox.setStandardButtons(
            QDialogButtonBox.Cancel | QDialogButtonBox.Save)
        verticalLayout.addWidget(buttonBox)
        buttonBox.accepted.connect(self.accept)
        buttonBox.rejected.connect(self.reject)

        self.boost_btn = QPushButton("Boost Confidence")
        self.boost_shortcut = QShortcut(QKeySequence('Space'), self)
        self.boost_shortcut.activated.connect(self.shortcutboost)
        self.boost_btn.clicked.connect(self.btnboost)
        verticalLayout.addWidget(self.boost_btn)

        self.delete_btn = QPushButton("Delete")
        self.delete_shortcut = QShortcut(QKeySequence('Delete'), self)
        self.delete_shortcut.activated.connect(self.shortcutdelete)
        self.delete_btn.clicked.connect(self.btndelete)
        verticalLayout.addWidget(self.delete_btn)
        self.initEditAnnot()

    def initEditAnnot(self):
        if self.exec():
            if self.delete:
                if self.trash_item in self.trash_items:
                    self.trash_items.remove(self.trash_item)
            else:
                if(self.trash_item.trash_type != self.updated_label):
                    self.updated_conf = 1
                self.trash_item.trash_type = self.updated_label
                self.trash_item.conf = self.updated_conf
                self.trash_item.updated = True

    def btnstate(self, b):
        if b.isChecked() == True:
            self.updated_label = self.classes.index(b.text())

    def valuechange(self):
        self.updated_conf = self.slider.value()

    def btndelete(self, b):
        self.delete = True
        self.accept()

    def shortcutdelete(self):
        self.delete = True
        self.accept()

    def btnboost(self, b):
        self.updated_conf = 1
        self.accept()

    def shortcutboost(self):
        self.updated_conf = 1
        self.accept()

    def shortcutc0(self):
        self.c0.click()
        self.accept()

    def shortcutc1(self):
        self.c1.click()
        self.accept()

    def shortcutc2(self):
        self.c2.click()
        self.accept()

    def shortcutc3(self):
        self.c3.click()
        self.accept()

