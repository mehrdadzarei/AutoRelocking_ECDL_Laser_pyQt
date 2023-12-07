######################################################################################################
# @author Mehrdad Zarei <mzarei@umk.pl>
# @date 2021.07.27
# @version 0
#
# @brief GUI for monitoring and re-locking laser
#
######################################################################################################



from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys, time
import threading
import socket
import pyqtgraph as pg
import numpy as np
from pyqtgraph.functions import mkPen
from scipy.signal import find_peaks
import wavelength_meter
from wlmConst import *
import laser_control

QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)     # for adjusting with diffrent windows scaling in different monitors


# path = 'D:\\programming\\KL FAMO\\HighFinesse\\'
path = 'C:\\Users\\UMK\\Documents\\cavity_progs\\py-ws7\\Mehrdad\\'



class Ui_MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        action = 'show'
        self.wlm = wavelength_meter.WavelengthMeter(WLM = 'W6', action = 'show')
        self.lc = laser_control.LaserControl()

        self.stop = cCtrlStopAll
        self.adjust = cCtrlStartAdjustment
        self.measurement = cCtrlStartMeasurement

        # [name, target frequency, PiezoRelockMode, CurrentRelockMode, update, portNamePiezo, portNameCurrent, portNameInput]
        self.chName = {'1': ['1: Blue', '325.2520', 0, 0, 0, '', '', ''], '2': ['2:', '0', 0, 0, 0, '', '', ''], '3': ['3:', '0', 0, 0, 0, '', '', ''],\
                       '4': ['4: TiSa', '368.5554', 0, 0, 0, '', '', ''], '5': ['5: Re-Pumper(679)', '441.3327', 0, 0, 0, '', '', ''],\
                       '6': ['6: Re-Pumper(707)', '423.9135', 0, 0, 0, '', '', ''], '7': ['7: Red Mot', '434.8291', 0, 0, 0, '', '', ''], \
                       '8': ['8: Clock', '429.2284', 0, 0, 0, '', '', '']}
        self.refData = {'1': [], '2': [], '3': [], '4': [],\
                       '5': [], '6': [], '7': [], '8': []}
        self.refDataInfo = {'1': [0, 0.0001, 0.00005, 0], '2': [0, 0.0001, 0.00005, 0], '3': [0, 0.0001, 0.00005, 0],\
                            '4': [0, 0.0001, 0.00005, 0], '5': [0, 0.00004, 0.00001, 0], '6': [0, 0.00004, 0.00001, 0], \
                            '7': [0, 0.0003, 0.00025, 0], '8': [0, 0.0003, 0.00025, 0]}      # [no peaks diff, freq_diff_thr, freq_diff_std, has_refData]
        self.fname = {'1': path + 'refCh1.txt', '2': path + 'refCh2.txt', '3': path + 'refCh3.txt', '4': path + 'refCh4.txt',\
                       '5': path + 'refCh5.txt', '6': path + 'refCh6.txt', '7': path + 'refCh7.txt', '8': path + 'refCh8.txt'}
        self.IParam = {'1': [-1.0, 1.0, 0, '+'], '2': [-1.0, 1.0, 0, '+'], '3': [-1.0, 1.0, 0, '+'], '4': [-1.0, 1.0, 0, '+'],\
                       '5': [-1.0, 0.01, 0, '+'], '6': [-1.0, 0.15, 0, '+'], '7': [-1.0, 1.0, 0, '+'], '8': [-1.0, 1.0, 0, '+']}      # [min, max, last_value, prev_state]
        self.PztParam = {'1': [-3.0, 3.0, 0, '+'], '2': [-3.0, 3.0, 0, '+'], '3': [-3.0, 3.0, 0, '+'], '4': [-3.0, 3.0, 0, '+'],\
                         '5': [-2.0, 3.0, 0, '+'], '6': [-3.0, 3.0, 0, '+'], '7': [-3.0, 3.0, 0, '+'], '8': [-3.0, 3.0, 0, '+']}      # [min, max, last_value, prev_state]

        self.range = 0
        self.lcState = False
        self.pztCurrVal = 0.0
        self.iCurrVal = 0.0
        self.currStep = 0.001
        self.PztStep = 0.001
        self.freqStep = 0.001       # 1 GHz
        self.freq_diff = 0.0
        self.noPeaks_diff = 0
        self.height_thr = 150
        self.distance_thr = 20
        self.noP_diff_thr = 8
        self.noP_diff_std = 7
        # self.freq_diff_thr = 0.0001
        # self.freq_diff_std = 0.00005

        self.updMode = False
        self.analMode = False
        self.relockMode = False
        self.noCorrCurr = 0
        self.noCorrPzt = 0
        self.noCavityCorrPzt = 0
        self.corr = False
        self.cavityLock = False
        self.updWidgetB = False
        self.updWidgetA = False
        self.monitor = True
        self.mode = True
        self.currTab = 0
        self.currOverTab = 0
        self.expoChang = False
        self.ch = self.wlm.getSwitcherChannel()
        self.fontNorm = QFont('Arial', 12)
        self.fontMode = QFont('Arial', 12, weight=75)
        PORT = 12346
        SERVER = "192.168.3.204"
        self.ADDR = (SERVER, PORT)
        self.FORMAT = "utf-8"

        self.createGUI()
        
        if action == 'show':
            self.highFinesse.setChecked(True)

        self.upData.setDisabled(True)
        self.downData.setDisabled(True)
        self.automaticExpo.setDisabled(True)

        self.pztRelock.setEnabled(False)
        self.currRelock.setEnabled(False)
        self.stRelock.setEnabled(False)

        self.setRef.setEnabled(False)
        
        self.start.setEnabled(True)
        
        self.currS.setDisabled(True)
        self.currSb.setDisabled(True)
        self.piezoS.setDisabled(True)
        self.piezoSb.setDisabled(True)

        try:
                
            with open(path + 'setting.txt', 'r') as f:

                a = f.readlines()
                for i in self.IParam:
                        
                    self.IParam[i][2] = float(a[a.index('channelI ' + i + ':\n') + 1])      # last value
                    self.IParam[i][3] = a[a.index('channelI ' + i + ':\n') + 2][:-1]        # prev state
                for i in self.PztParam:
                        
                    self.PztParam[i][2] = float(a[a.index('channelPzt ' + i + ':\n') + 1])  # last value
                    self.PztParam[i][3] = a[a.index('channelPzt ' + i + ':\n') + 2][:-1]    # prev state
                for i in self.chName:
                        
                    self.chName[i][2] = int(a[a.index('properties ' + i + ':\n') + 1])      # pzt relock mode
                    self.chName[i][3] = int(a[a.index('properties ' + i + ':\n') + 2])      # curr relock mode
                    self.chName[i][4] = int(a[a.index('properties ' + i + ':\n') + 3])      # update
                    self.chName[i][5] = a[a.index('properties ' + i + ':\n') + 4][:-1]      # piezo port name
                    self.chName[i][6] = a[a.index('properties ' + i + ':\n') + 5][:-1]      # current port name
                    self.chName[i][7] = a[a.index('properties ' + i + ':\n') + 6][:-1]      # input port name
                
                self.updTimeData.setValue(int(a[a.index('timig:\n') + 1]))            # min
                self.relockTimeData.setValue(int(a[a.index('timig:\n') + 2]))        # s
                self.stpRelockTimeData.setValue(int(a[a.index('timig:\n') + 3]))     # s
                self.switchTimeData.setValue(int(a[a.index('timig:\n') + 4]))       # ms

                f.close()
        except IOError:
            pass

        for i in self.refData:
            
            if self.readFile(i):
                
                self.refDataInfo[i][3] = 1
                peaksIndRef, peaksHRef = find_peaks(self.refData[i], height = self.height_thr, distance = self.distance_thr)
                self.refDataInfo[i][0] = len(peaksIndRef)

        self.setUpd()

    def createGUI(self):

        self.setWindowTitle('Re-Locking')
        self.resize(1100,700)
        # self.setGeometry(100, 100, 1150, 600)
        # self.showMaximized()
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        QApplication.setStyle('Fusion')
        # background-color: black; color: white; 
        # self.setStyleSheet('font-size: 11pt;')
        icon = QIcon()
        icon.addPixmap(QPixmap(path + "icon\\main.png"), QIcon.Normal, QIcon.Off)
        self.setWindowIcon(icon)
        
        self.initialTabs = QTabWidget()
        homeTab = QWidget()
        overTab = QWidget()
        self.initialTabs.setDocumentMode(True)
        # self.initialTabs.setFixedHeight(200)
        self.initialTabs.setFont(self.fontNorm)

        self.initialTabs.addTab(homeTab, "Home")
        self.initialTabs.addTab(overTab, "Overview")

        homeTab.layout = QVBoxLayout()
        overTab.layout = QVBoxLayout()

        # centralWidget = QWidget()
        # mainLayout = QVBoxLayout()

        # creating home tab
        self.createElementsOfInfo()
        self.createElementsOfLC()
        self.createElementsOfGraph()
        self.createElementsOfTabs()

        self.initialTabs.currentChanged.connect(self.setOverview)
        self.chData.activated[str].connect(self.changeChannel)
        self.pztRelock.stateChanged.connect(self.pztRelockState)
        self.currRelock.stateChanged.connect(self.currRelockState)
        self.stRelock.clicked.connect(self.stRelockState)
        self.setRef.clicked.connect(self.setReference)
        self.automaticExpo.stateChanged.connect(self.expoAutomatic)
        self.upData.valueChanged.connect(self.exposureUp)
        # self.upData.editingFinished.connect(self.exposureUp)
        self.downData.valueChanged.connect(self.exposureDown)
        self.highFinesse.stateChanged.connect(self.showHighFinesse)
        self.start.clicked.connect(self.operation)
        self.plotPattern.getPlotItem().sigRangeChanged.connect(self.setPlotRange)
        self.currS.valueChanged.connect(self.lc_current)
        self.currSb.valueChanged.connect(self.lc_currentB)
        self.piezoS.valueChanged.connect(self.lc_piezo)
        self.piezoSb.valueChanged.connect(self.lc_piezoB)
        self.tabs.currentChanged.connect(self.setSwitchMode)

        # homeTab.layout.setAlignment(Qt.AlignTop)
        homeTab.layout.setSpacing(10)
        homeTab.layout.addSpacing(10)
        homeTab.layout.addLayout(self.infoBox)
        homeTab.layout.addSpacing(10)
        homeTab.layout.addLayout(self.lcBox)
        homeTab.layout.addSpacing(10)
        homeTab.layout.addLayout(self.plotBox)
        homeTab.layout.addSpacing(10)
        homeTab.layout.addWidget(self.tabs)
        homeTab.layout.addSpacing(10)

        # creating overview tab
        self.createElementsOfOverview()

        self.pzt1.stateChanged.connect(self.pzt1RelockStateDetails)
        self.curr1.stateChanged.connect(self.curr1RelockStateDetails)
        self.portNamePztData1.textChanged.connect(self.portChangePzt1)
        self.portNameCurrData1.textChanged.connect(self.portChangeCurr1)
        self.portNameInpData1.textChanged.connect(self.portChangeInp1)

        self.pzt2.stateChanged.connect(self.pzt2RelockStateDetails)
        self.curr2.stateChanged.connect(self.curr2RelockStateDetails)
        self.portNamePztData2.textChanged.connect(self.portChangePzt2)
        self.portNameCurrData2.textChanged.connect(self.portChangeCurr2)
        self.portNameInpData2.textChanged.connect(self.portChangeInp2)
        
        self.pzt3.stateChanged.connect(self.pzt3RelockStateDetails)
        self.curr3.stateChanged.connect(self.curr3RelockStateDetails)
        self.portNamePztData3.textChanged.connect(self.portChangePzt3)
        self.portNameCurrData3.textChanged.connect(self.portChangeCurr3)
        self.portNameInpData3.textChanged.connect(self.portChangeInp3)
        
        self.pzt4.stateChanged.connect(self.pzt4RelockStateDetails)
        self.curr4.stateChanged.connect(self.curr4RelockStateDetails)
        self.portNamePztData4.textChanged.connect(self.portChangePzt4)
        self.portNameCurrData4.textChanged.connect(self.portChangeCurr4)
        self.portNameInpData4.textChanged.connect(self.portChangeInp4)
        
        self.pzt5.stateChanged.connect(self.pzt5RelockStateDetails)
        self.curr5.stateChanged.connect(self.curr5RelockStateDetails)
        self.portNamePztData5.textChanged.connect(self.portChangePzt5)
        self.portNameCurrData5.textChanged.connect(self.portChangeCurr5)
        self.portNameInpData5.textChanged.connect(self.portChangeInp5)
        
        self.pzt6.stateChanged.connect(self.pzt6RelockStateDetails)
        self.curr6.stateChanged.connect(self.curr6RelockStateDetails)
        self.portNamePztData6.textChanged.connect(self.portChangePzt6)
        self.portNameCurrData6.textChanged.connect(self.portChangeCurr6)
        self.portNameInpData6.textChanged.connect(self.portChangeInp6)
        
        self.pzt7.stateChanged.connect(self.pzt7RelockStateDetails)
        self.curr7.stateChanged.connect(self.curr7RelockStateDetails)
        self.portNamePztData7.textChanged.connect(self.portChangePzt7)
        self.portNameCurrData7.textChanged.connect(self.portChangeCurr7)
        self.portNameInpData7.textChanged.connect(self.portChangeInp7)
        
        self.pzt8.stateChanged.connect(self.pzt8RelockStateDetails)
        self.curr8.stateChanged.connect(self.curr8RelockStateDetails)
        self.portNamePztData8.textChanged.connect(self.portChangePzt8)
        self.portNameCurrData8.textChanged.connect(self.portChangeCurr8)
        self.portNameInpData8.textChanged.connect(self.portChangeInp8)

        overTab.layout.setSpacing(10)
        overTab.layout.addSpacing(10)
        overTab.layout.addLayout(self.setting)
        overTab.layout.addSpacing(10)
        overTab.layout.addLayout(self.relockInfo)
        overTab.layout.addSpacing(10)
        
        homeTab.setLayout(homeTab.layout)
        overTab.setLayout(overTab.layout)
        # centralWidget.setLayout(mainLayout)
        self.setCentralWidget(self.initialTabs)
    
    def createElementsOfInfo(self):

        self.infoBox = QGridLayout()
        self.infoBox.setSpacing(10)
        
        ch = QLabel("Channel")
        ch.setFont(self.fontNorm)
        self.chData = QComboBox()
        self.chData.setFont(self.fontNorm)
        self.chData.installEventFilter(self)
        items = [self.chName[i][0] for i in self.chName]
        self.chData.addItems(items)
        
        expo = QLabel("Exposure [ms]")
        expo.setFont(self.fontNorm)
        self.automaticExpo = QCheckBox("Automatic")
        self.automaticExpo.setFont(self.fontNorm)
        up = QLabel("1")
        up.setFont(self.fontNorm)
        down = QLabel("2+")
        down.setFont(self.fontNorm)
        self.upData = QSpinBox()
        self.upData.setFont(self.fontNorm)
        self.upData.installEventFilter(self)
        self.upData.setMinimum(1)
        self.upData.setMaximum(9999)
        self.downData = QSpinBox()
        self.downData.setFont(self.fontNorm)
        self.downData.installEventFilter(self)
        self.downData.setMinimum(0)
        self.downData.setMaximum(9999)

        self.pztRelock = QCheckBox("Piezo Relock")
        self.pztRelock.setFont(self.fontNorm)
        # self.pztRelock.setChecked(True)

        self.currRelock = QCheckBox("Current Relock")
        self.currRelock.setFont(self.fontNorm)

        self.stRelock = QPushButton("Start Relocking")
        self.stRelock.setFont(self.fontNorm)
        self.stRelock.setDefault(True)

        self.setRef = QPushButton("Set Ref")
        self.setRef.setFont(self.fontNorm)
        self.setRef.setDefault(True)

        self.infoMsg = QLabel("Run...")
        self.infoMsg.setFont(QFont('Arial', 20, QFont.Bold))
        self.infoMsg.setStyleSheet("color: rgb(255, 0, 0)")

        self.highFinesse = QCheckBox("HighFinesse")
        self.highFinesse.setFont(self.fontNorm)

        self.start = QPushButton("Start")
        self.start.setFont(self.fontNorm)
        self.start.setDefault(True)
        
        self.infoBox.addWidget(ch, 0, 0, 1, 1)
        self.infoBox.addWidget(self.chData, 0, 1, 1, 1)
        
        self.infoBox.setColumnMinimumWidth(2, 50)

        self.infoBox.addWidget(expo, 0, 4, 1, 1)
        self.infoBox.addWidget(self.automaticExpo, 1, 4, 1, 1)
        self.infoBox.addWidget(up, 0, 5, 1, 1)
        self.infoBox.addWidget(down, 1, 5, 1, 1)
        self.infoBox.addWidget(self.upData, 0, 6, 1, 1)
        self.infoBox.addWidget(self.downData, 1, 6, 1, 1)

        self.infoBox.setColumnMinimumWidth(7, 50)

        self.infoBox.addWidget(self.pztRelock, 0, 8, 1, 1)
        self.infoBox.addWidget(self.currRelock, 1, 8, 1, 1)

        self.infoBox.addWidget(self.stRelock, 0, 9, 1, 1)
        self.infoBox.addWidget(self.setRef, 1, 9, 1, 1)

        self.infoBox.setColumnStretch(10, 1)

        self.infoBox.addWidget(self.infoMsg, 0, 11, 2, 2)
        
        self.infoBox.setColumnStretch(13, 1)

        self.infoBox.addWidget(self.highFinesse, 0, 14, 1, 1)

        self.infoBox.addWidget(self.start, 1, 14, 1, 1)

    def createElementsOfLC(self):

        self.lcBox = QGridLayout()
        self.lcBox.setSpacing(10)

        l_c = QLabel("Laser Control (Manually)")
        l_c.setFont(self.fontNorm)

        # cur = QLabel("Current: ")
        # cur.setFont(self.fontNorm)

        self.currVal = QLabel("Current")
        self.currVal.setFont(self.fontNorm)
        
        self.currS = QSlider(Qt.Horizontal)
        self.currS.installEventFilter(self)
        self.currS.setTickPosition(QSlider.TicksBelow)
        self.currS.setTickInterval(50)
        self.currS.setSingleStep(1)

        self.currSb = QDoubleSpinBox()
        self.currSb.setFont(self.fontNorm)
        self.currSb.installEventFilter(self)
        self.currSb.setDecimals(3)
        self.currSb.setSingleStep(self.currStep)
        self.currSb.setFixedWidth(100)
        
        # piez = QLabel("Piezo: ")
        # piez.setFont(self.fontNorm)
        
        self.pztVal = QLabel("Piezo")
        self.pztVal.setFont(self.fontNorm)

        self.piezoS = QSlider(Qt.Horizontal)
        self.piezoS.installEventFilter(self)
        self.piezoS.setTickPosition(QSlider.TicksBelow)
        self.piezoS.setTickInterval(50)
        self.piezoS.setSingleStep(1)

        self.piezoSb = QDoubleSpinBox()
        self.piezoSb.setFont(self.fontNorm)
        self.piezoSb.installEventFilter(self)
        self.piezoSb.setDecimals(3)
        self.piezoSb.setSingleStep(self.PztStep)
        self.piezoSb.setFixedWidth(100)

        self.lcBox.addWidget(l_c, 1, 0, 1, 1)

        self.lcBox.setColumnMinimumWidth(1, 50)

        # self.lcBox.addWidget(cur, 0, 3, 1, 1)
        self.lcBox.addWidget(self.currVal, 0, 3, 1, 1)
        self.lcBox.addWidget(self.currS, 1, 2, 1, 3)
        self.lcBox.addWidget(self.currSb, 1, 5, 1, 1)

        self.lcBox.setColumnMinimumWidth(6, 50)

        # self.lcBox.addWidget(piez, 0, 8, 1, 1)
        self.lcBox.addWidget(self.pztVal, 0, 8, 1, 1)
        self.lcBox.addWidget(self.piezoS, 1, 7, 1, 3)
        self.lcBox.addWidget(self.piezoSb, 1, 10, 1, 1)

    def createElementsOfGraph(self):

        self.plotBox = QGridLayout()

        self.plotPattern = pg.PlotWidget(title="Interferometer")
        # self.plotPattern.setLabels(left = ('Amplitude'), bottom = ('data points'))
        # self.plotPattern.addLegend(offset = (0, -140))
        self.plotPattern.plotItem.clear()
        self.plotPattern.plotItem.getViewBox().setMouseEnabled(x=False, y=False)
        self.plotPattern.plotItem.showGrid(x=True, y=True, alpha=0.2)
        self.plotPattern.plotItem.getViewBox().setMenuEnabled(False)
        self.plotPattern.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
        self.plotPattern.plotItem.getViewBox().setBorder()
        self.plotPattern.plotItem.hideButtons()
        # self.plotPattern.plotItem.showButtons()
        # self.plotPattern.getPlotItem().autoBtn.disable()
        # self.plotPattern.getPlotItem().autoBtn.enabled.__bool__()
        self.data = self.plotPattern.plotItem.plot(pen = mkPen('r', width = 3))
        self.data1 = self.plotPattern.plotItem.plot(pen = mkPen(color=(0, 0, 255), width = 3))
        self.data2 = self.plotPattern.plotItem.plot(pen = mkPen(color=(128, 128, 128), width = 3))
        self.data3 = self.plotPattern.plotItem.plot(pen = mkPen(color=(255, 0, 255), width = 3))
        self.data4 = self.plotPattern.plotItem.plot(pen = mkPen(color=(255, 102, 102), width = 3))
        self.data5 = self.plotPattern.plotItem.plot(pen = mkPen(color=(153, 51, 255), width = 3))
        self.data6 = self.plotPattern.plotItem.plot(pen = mkPen(color=(204, 204, 0), width = 3))
        self.data7 = self.plotPattern.plotItem.plot(pen = mkPen(color=(255, 0, 0), width = 3))
        self.data8 = self.plotPattern.plotItem.plot(pen = mkPen(color=(0, 102, 0), width = 3))

        # self.plotStd = pg.PlotWidget(title = "Residual")
        # # self.plotStd.setLabels(left = ('Amplitude'), bottom = ('data points'))
        # # self.plotStd.addLegend()
        # self.plotStd.scale(1, 3)
        # self.plotStd.plotItem.clear()
        # # self.plotStd.plotItem.getViewBox().setMouseEnabled(x=False, y=False)
        # self.plotStd.plotItem.showGrid(x=True, y=True)
        # # self.plotStd.plotItem.getViewBox().setMenuEnabled(False)
        # # self.plotStd.plotItem.getViewBox().setMouseMode(pg.ViewBox.RectMode)
        # self.plotStd.plotItem.getViewBox().setBorder()
        # self.res = self.plotStd.plotItem.plot(pen = (0, 250, 0))

        self.plotBox.addWidget(self.plotPattern, 0, 0, 1, 1)
        self.plotBox.setRowStretch(0, 1)
        self.plotBox.setRowMinimumHeight(0, 300)
        # self.plotBox.addWidget(self.plotStd, 1, 0, 1, 1)
        # self.plotBox.setRowMinimumHeight(1, 100)

    def createElementsOfTabs(self):

        self.tabs = QTabWidget()
        mainTab = QWidget()
        switchTab = QWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setFixedHeight(200)
        self.tabs.setFont(self.fontNorm)

        self.tabs.addTab(mainTab, "Main")
        self.tabs.addTab(switchTab, "Switch Mode")

        mainTab.layout = QGridLayout()
        switchTab.layout = QGridLayout()

        mainTab.layout.setSpacing(10)
        switchTab.layout.setSpacing(10)
        
        # create main Tab
        wave = QLabel("Wavelength: ")
        wave.setFont(QFont('Arial', 28, QFont.Bold))
        self.waveData = QLabel("")
        self.waveData.setFont(QFont('Arial', 28, QFont.Bold))
        self.waveData.setFixedWidth(250)
        
        freq = QLabel("Frequency: ")
        freq.setFont(QFont('Arial', 28, QFont.Bold))
        self.freqData = QLabel("")
        self.freqData.setFont(QFont('Arial', 28, QFont.Bold))
        self.freqData.setFixedWidth(250)
        
        tgf = QLabel("Target: ")
        tgf.setFont(QFont('Arial', 28, QFont.Bold))
        self.tgfData = QLabel("")
        self.tgfData.setFont(QFont('Arial', 28, QFont.Bold))
        self.tgfData.setFixedWidth(250)

        mainTab.layout.setColumnStretch(0, 1)

        mainTab.layout.addWidget(wave, 0, 1, 1, 1)
        mainTab.layout.addWidget(self.waveData, 0, 2, 1, 1)
        
        mainTab.layout.addWidget(freq, 1, 1, 1, 1)
        mainTab.layout.addWidget(self.freqData, 1, 2, 1, 1)
        
        mainTab.layout.addWidget(tgf, 2, 1, 1, 1)
        mainTab.layout.addWidget(self.tgfData, 2, 2, 1, 1)

        mainTab.layout.setColumnStretch(3, 1)

        # create switch-mode Tab
        self.groupBox1 = QGroupBox(self.chName['1'][0])
        self.groupBox1.setCheckable(True)
        self.groupBox1.setChecked(False)
        self.groupBox1.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBox1.setFont(self.fontMode)
        box1 = QGridLayout()

        self.show1 = QCheckBox("Show")
        self.show1.setFont(self.fontMode)

        wave1 = QLabel("Wavelength: ")
        wave1.setFont(self.fontMode)
        self.waveData1 = QLabel("")
        self.waveData1.setFont(self.fontMode)
        
        freq1 = QLabel("Frequency: ")
        freq1.setFont(self.fontMode)
        self.freqData1 = QLabel("")
        self.freqData1.setFont(self.fontMode)

        box1.setColumnStretch(0, 1)
        box1.addWidget(self.show1, 0, 1, 1, 1)
        box1.addWidget(wave1, 1, 1, 1, 1)
        box1.addWidget(self.waveData1, 1, 2, 1, 1)
        box1.addWidget(freq1, 2, 1, 1, 1)
        box1.addWidget(self.freqData1, 2, 2, 1, 1)
        box1.setColumnStretch(3, 1)

        self.groupBox1.setLayout(box1)

        self.groupBox2 = QGroupBox(self.chName['2'][0])
        self.groupBox2.setCheckable(True)
        self.groupBox2.setChecked(False)
        self.groupBox2.setStyleSheet("color: rgb(128, 128, 128)")
        self.groupBox2.setFont(self.fontMode)
        box2 = QGridLayout()

        self.show2 = QCheckBox("Show")
        self.show2.setFont(self.fontMode)

        wave2 = QLabel("Wavelength: ")
        wave2.setFont(self.fontMode)
        self.waveData2 = QLabel("")
        self.waveData2.setFont(self.fontMode)
        
        freq2 = QLabel("Frequency: ")
        freq2.setFont(self.fontMode)
        self.freqData2 = QLabel("")
        self.freqData2.setFont(self.fontMode)

        box2.setColumnStretch(0, 1)
        box2.addWidget(self.show2, 0, 1, 1, 1)
        box2.addWidget(wave2, 1, 1, 1, 1)
        box2.addWidget(self.waveData2, 1, 2, 1, 1)
        box2.addWidget(freq2, 2, 1, 1, 1)
        box2.addWidget(self.freqData2, 2, 2, 1, 1)
        box2.setColumnStretch(3, 1)

        self.groupBox2.setLayout(box2)

        self.groupBox3 = QGroupBox(self.chName['3'][0])
        self.groupBox3.setCheckable(True)
        self.groupBox3.setChecked(False)
        self.groupBox3.setStyleSheet("color: rgb(255, 0, 255)")
        self.groupBox3.setFont(self.fontMode)
        box3 = QGridLayout()

        self.show3 = QCheckBox("Show")
        self.show3.setFont(self.fontMode)

        wave3 = QLabel("Wavelength: ")
        wave3.setFont(self.fontMode)
        self.waveData3 = QLabel("")
        self.waveData3.setFont(self.fontMode)
        
        freq3 = QLabel("Frequency: ")
        freq3.setFont(self.fontMode)
        self.freqData3 = QLabel("")
        self.freqData3.setFont(self.fontMode)

        box3.setColumnStretch(0, 1)
        box3.addWidget(self.show3, 0, 1, 1, 1)
        box3.addWidget(wave3, 1, 1, 1, 1)
        box3.addWidget(self.waveData3, 1, 2, 1, 1)
        box3.addWidget(freq3, 2, 1, 1, 1)
        box3.addWidget(self.freqData3, 2, 2, 1, 1)
        box3.setColumnStretch(3, 1)

        self.groupBox3.setLayout(box3)

        self.groupBox4 = QGroupBox(self.chName['4'][0])
        self.groupBox4.setCheckable(True)
        self.groupBox4.setChecked(False)
        self.groupBox4.setStyleSheet("color: rgb(255, 102, 102)")
        self.groupBox4.setFont(self.fontMode)
        box4 = QGridLayout()

        self.show4 = QCheckBox("Show")
        self.show4.setFont(self.fontMode)

        wave4 = QLabel("Wavelength: ")
        wave4.setFont(self.fontMode)
        self.waveData4 = QLabel("")
        self.waveData4.setFont(self.fontMode)
        
        freq4 = QLabel("Frequency: ")
        freq4.setFont(self.fontMode)
        self.freqData4 = QLabel("")
        self.freqData4.setFont(self.fontMode)

        box4.setColumnStretch(0, 1)
        box4.addWidget(self.show4, 0, 1, 1, 1)
        box4.addWidget(wave4, 1, 1, 1, 1)
        box4.addWidget(self.waveData4, 1, 2, 1, 1)
        box4.addWidget(freq4, 2, 1, 1, 1)
        box4.addWidget(self.freqData4, 2, 2, 1, 1)
        box4.setColumnStretch(3, 1)

        self.groupBox4.setLayout(box4)

        self.groupBox5 = QGroupBox(self.chName['5'][0])
        self.groupBox5.setCheckable(True)
        self.groupBox5.setChecked(False)
        self.groupBox5.setStyleSheet("color: rgb(153, 51, 255)")
        self.groupBox5.setFont(self.fontMode)
        box5 = QGridLayout()

        self.show5 = QCheckBox("Show")
        self.show5.setFont(self.fontMode)

        wave5 = QLabel("Wavelength: ")
        wave5.setFont(self.fontMode)
        self.waveData5 = QLabel("")
        self.waveData5.setFont(self.fontMode)
        
        freq5 = QLabel("Frequency: ")
        freq5.setFont(self.fontMode)
        self.freqData5 = QLabel("")
        self.freqData5.setFont(self.fontMode)

        box5.setColumnStretch(0, 1)
        box5.addWidget(self.show5, 0, 1, 1, 1)
        box5.addWidget(wave5, 1, 1, 1, 1)
        box5.addWidget(self.waveData5, 1, 2, 1, 1)
        box5.addWidget(freq5, 2, 1, 1, 1)
        box5.addWidget(self.freqData5, 2, 2, 1, 1)
        box5.setColumnStretch(3, 1)

        self.groupBox5.setLayout(box5)

        self.groupBox6 = QGroupBox(self.chName['6'][0])
        self.groupBox6.setCheckable(True)
        self.groupBox6.setChecked(False)
        self.groupBox6.setStyleSheet("color: rgb(204, 204, 0)")
        self.groupBox6.setFont(self.fontMode)
        box6 = QGridLayout()

        self.show6 = QCheckBox("Show")
        self.show6.setFont(self.fontMode)

        wave6 = QLabel("Wavelength: ")
        wave6.setFont(self.fontMode)
        self.waveData6 = QLabel("")
        self.waveData6.setFont(self.fontMode)
        
        freq6 = QLabel("Frequency: ")
        freq6.setFont(self.fontMode)
        self.freqData6 = QLabel("")
        self.freqData6.setFont(self.fontMode)

        box6.setColumnStretch(0, 1)
        box6.addWidget(self.show6, 0, 1, 1, 1)
        box6.addWidget(wave6, 1, 1, 1, 1)
        box6.addWidget(self.waveData6, 1, 2, 1, 1)
        box6.addWidget(freq6, 2, 1, 1, 1)
        box6.addWidget(self.freqData6, 2, 2, 1, 1)
        box6.setColumnStretch(3, 1)

        self.groupBox6.setLayout(box6)

        self.groupBox7 = QGroupBox(self.chName['7'][0])
        self.groupBox7.setCheckable(True)
        self.groupBox7.setChecked(False)
        self.groupBox7.setStyleSheet("color: rgb(255, 0, 0)")
        self.groupBox7.setFont(self.fontMode)
        box7 = QGridLayout()

        self.show7 = QCheckBox("Show")
        self.show7.setFont(self.fontMode)

        wave7 = QLabel("Wavelength: ")
        wave7.setFont(self.fontMode)
        self.waveData7 = QLabel("")
        self.waveData7.setFont(self.fontMode)
        
        freq7 = QLabel("Frequency: ")
        freq7.setFont(self.fontMode)
        self.freqData7 = QLabel("")
        self.freqData7.setFont(self.fontMode)

        box7.setColumnStretch(0, 1)
        box7.addWidget(self.show7, 0, 1, 1, 1)
        box7.addWidget(wave7, 1, 1, 1, 1)
        box7.addWidget(self.waveData7, 1, 2, 1, 1)
        box7.addWidget(freq7, 2, 1, 1, 1)
        box7.addWidget(self.freqData7, 2, 2, 1, 1)
        box7.setColumnStretch(3, 1)

        self.groupBox7.setLayout(box7)

        self.groupBox8 = QGroupBox(self.chName['8'][0])
        self.groupBox8.setCheckable(True)
        self.groupBox8.setChecked(False)
        self.groupBox8.setStyleSheet("color: rgb(0, 102, 0)")
        self.groupBox8.setFont(self.fontMode)
        box8 = QGridLayout()

        self.show8 = QCheckBox("Show")
        self.show8.setFont(self.fontMode)

        wave8 = QLabel("Wavelength: ")
        wave8.setFont(self.fontMode)
        self.waveData8 = QLabel("")
        self.waveData8.setFont(self.fontMode)
        
        freq8 = QLabel("Frequency: ")
        freq8.setFont(self.fontMode)
        self.freqData8 = QLabel("")
        self.freqData8.setFont(self.fontMode)

        box8.setColumnStretch(0, 1)
        box8.addWidget(self.show8, 0, 1, 1, 1)
        box8.addWidget(wave8, 1, 1, 1, 1)
        box8.addWidget(self.waveData8, 1, 2, 1, 1)
        box8.addWidget(freq8, 2, 1, 1, 1)
        box8.addWidget(self.freqData8, 2, 2, 1, 1)
        box8.setColumnStretch(3, 1)

        self.groupBox8.setLayout(box8)

        switchTab.layout.addWidget(self.groupBox1, 0, 0, 1, 1)
        switchTab.layout.addWidget(self.groupBox2, 0, 1, 1, 1)
        switchTab.layout.addWidget(self.groupBox3, 0, 3, 1, 1)
        switchTab.layout.addWidget(self.groupBox4, 0, 4, 1, 1)
        switchTab.layout.addWidget(self.groupBox5, 1, 0, 1, 1)
        switchTab.layout.addWidget(self.groupBox6, 1, 1, 1, 1)
        switchTab.layout.addWidget(self.groupBox7, 1, 3, 1, 1)
        switchTab.layout.addWidget(self.groupBox8, 1, 4, 1, 1)

        mainTab.setLayout(mainTab.layout)
        switchTab.setLayout(switchTab.layout)

    def createElementsOfOverview(self):

        self.setting = QGridLayout()
        self.relockInfo = QGridLayout()

        self.setting.setSpacing(10)
        self.relockInfo.setSpacing(10)
        
        updTime = QLabel("Update time (min)")
        updTime.setFont(self.fontNorm)
        self.updTimeData = QSpinBox()
        self.updTimeData.setFont(self.fontNorm)
        self.updTimeData.installEventFilter(self)
        self.updTimeData.setMinimum(0)
        self.updTimeData.setMaximum(9999)

        relockTime = QLabel("Relock time (s)")
        relockTime.setFont(self.fontNorm)
        self.relockTimeData = QSpinBox()
        self.relockTimeData.setFont(self.fontNorm)
        self.relockTimeData.installEventFilter(self)
        self.relockTimeData.setMinimum(1)
        self.relockTimeData.setMaximum(9999)

        stpRelockTime = QLabel("Stop Relock time (s)")
        stpRelockTime.setFont(self.fontNorm)
        self.stpRelockTimeData = QSpinBox()
        self.stpRelockTimeData.setFont(self.fontNorm)
        self.stpRelockTimeData.installEventFilter(self)
        self.stpRelockTimeData.setMinimum(1)
        self.stpRelockTimeData.setMaximum(9999)
        
        switchTime = QLabel("Switch Mode time (ms)")
        switchTime.setFont(self.fontNorm)
        self.switchTimeData = QSpinBox()
        self.switchTimeData.setFont(self.fontNorm)
        self.switchTimeData.installEventFilter(self)
        self.switchTimeData.setMinimum(300)
        self.switchTimeData.setMaximum(9999)

        self.setting.addWidget(updTime, 0, 0, 1, 1)
        self.setting.addWidget(self.updTimeData, 1, 0, 1, 1)

        self.setting.setColumnMinimumWidth(1, 50)
        
        self.setting.addWidget(relockTime, 0, 2, 1, 1)
        self.setting.addWidget(self.relockTimeData, 1, 2, 1, 1)

        self.setting.setColumnMinimumWidth(3, 50)
        
        self.setting.addWidget(stpRelockTime, 0, 4, 1, 1)
        self.setting.addWidget(self.stpRelockTimeData, 1, 4, 1, 1)

        self.setting.setColumnMinimumWidth(5, 50)
        
        self.setting.addWidget(switchTime, 0, 6, 1, 1)
        self.setting.addWidget(self.switchTimeData, 1, 6, 1, 1)
        
        self.setting.setColumnStretch(7, 1)

        h = 300
        self.groupBoxR1 = QGroupBox(self.chName['1'][0])
        self.groupBoxR1.setCheckable(True)
        self.groupBoxR1.setChecked(True)
        # self.groupBoxR1.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR1.setFont(self.fontMode)
        self.groupBoxR1.setMaximumHeight(h)
        box1 = QGridLayout()

        self.pzt1 = QCheckBox("Piezo Relock")
        self.pzt1.setFont(self.fontMode)

        self.curr1 = QCheckBox("Current Relock")
        self.curr1.setFont(self.fontMode)

        freq1 = QLabel("Frequency: ")
        freq1.setFont(self.fontMode)
        self.freqDataR1 = QLabel("")
        self.freqDataR1.setFont(self.fontMode)

        trg1 = QLabel("Target: ")
        trg1.setFont(self.fontMode)
        trgData1 = QLabel(self.chName[str(1)][1] + " THz")
        trgData1.setFont(self.fontMode)

        mode1 = QLabel("Mode: ")
        mode1.setFont(self.fontMode)
        self.modeData1 = QLabel("")
        self.modeData1.setFont(self.fontMode)

        portNamePzt1 = QLabel("Port Name Piezo: ")
        portNamePzt1.setFont(self.fontMode)
        self.portNamePztData1 = QLineEdit()
        self.portNamePztData1.setFont(self.fontMode)
        self.portNamePztData1.setPlaceholderText('Dev1/ao0')

        portNameCurr1 = QLabel("Port Name Current: ")
        portNameCurr1.setFont(self.fontMode)
        self.portNameCurrData1 = QLineEdit()
        self.portNameCurrData1.setFont(self.fontMode)
        self.portNameCurrData1.setPlaceholderText('Dev1/ao0')

        portNameInp1 = QLabel("Port Name Input: ")
        portNameInp1.setFont(self.fontMode)
        self.portNameInpData1 = QLineEdit()
        self.portNameInpData1.setFont(self.fontMode)
        self.portNameInpData1.setPlaceholderText('Dev1/ai0')

        box1.setColumnStretch(0, 1)
        box1.addWidget(self.pzt1, 0, 1, 1, 1)
        box1.addWidget(self.curr1, 1, 1, 1, 1)
        box1.addWidget(freq1, 2, 1, 1, 1)
        box1.addWidget(self.freqDataR1, 2, 2, 1, 1)
        box1.addWidget(trg1, 3, 1, 1, 1)
        box1.addWidget(trgData1, 3, 2, 1, 1)
        box1.addWidget(mode1, 4, 1, 1, 1)
        box1.addWidget(self.modeData1, 4, 2, 1, 1)
        box1.addWidget(portNamePzt1, 5, 1, 1, 1)
        box1.addWidget(self.portNamePztData1, 5, 2, 1, 1)
        box1.addWidget(portNameCurr1, 6, 1, 1, 1)
        box1.addWidget(self.portNameCurrData1, 6, 2, 1, 1)
        box1.addWidget(portNameInp1, 7, 1, 1, 1)
        box1.addWidget(self.portNameInpData1, 7, 2, 1, 1)
        box1.setColumnStretch(8, 1)

        self.groupBoxR1.setLayout(box1)

        self.groupBoxR2 = QGroupBox(self.chName['2'][0])
        self.groupBoxR2.setCheckable(True)
        self.groupBoxR2.setChecked(True)
        # self.groupBoxR2.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR2.setFont(self.fontMode)
        self.groupBoxR2.setMaximumHeight(h)
        box2 = QGridLayout()

        self.pzt2 = QCheckBox("Piezo Relock")
        self.pzt2.setFont(self.fontMode)

        self.curr2 = QCheckBox("Current Relock")
        self.curr2.setFont(self.fontMode)

        freq2 = QLabel("Frequency: ")
        freq2.setFont(self.fontMode)
        self.freqDataR2 = QLabel("")
        self.freqDataR2.setFont(self.fontMode)

        trg2 = QLabel("Target: ")
        trg2.setFont(self.fontMode)
        trgData2 = QLabel(self.chName[str(2)][1] + " THz")
        trgData2.setFont(self.fontMode)

        mode2 = QLabel("Mode: ")
        mode2.setFont(self.fontMode)
        self.modeData2 = QLabel("")
        self.modeData2.setFont(self.fontMode)

        portNamePzt2 = QLabel("Port Name Piezo: ")
        portNamePzt2.setFont(self.fontMode)
        self.portNamePztData2 = QLineEdit()
        self.portNamePztData2.setFont(self.fontMode)
        self.portNamePztData2.setPlaceholderText('Dev1/ao0')

        portNameCurr2 = QLabel("Port Name Current: ")
        portNameCurr2.setFont(self.fontMode)
        self.portNameCurrData2 = QLineEdit()
        self.portNameCurrData2.setFont(self.fontMode)
        self.portNameCurrData2.setPlaceholderText('Dev1/ao0')

        portNameInp2 = QLabel("Port Name Input: ")
        portNameInp2.setFont(self.fontMode)
        self.portNameInpData2 = QLineEdit()
        self.portNameInpData2.setFont(self.fontMode)
        self.portNameInpData2.setPlaceholderText('Dev1/ai0')

        box2.setColumnStretch(0, 1)
        box2.addWidget(self.pzt2, 0, 1, 1, 1)
        box2.addWidget(self.curr2, 1, 1, 1, 1)
        box2.addWidget(freq2, 2, 1, 1, 1)
        box2.addWidget(self.freqDataR2, 2, 2, 1, 1)
        box2.addWidget(trg2, 3, 1, 1, 1)
        box2.addWidget(trgData2, 3, 2, 1, 1)
        box2.addWidget(mode2, 4, 1, 1, 1)
        box2.addWidget(self.modeData2, 4, 2, 1, 1)
        box2.addWidget(portNamePzt2, 5, 1, 1, 1)
        box2.addWidget(self.portNamePztData2, 5, 2, 1, 1)
        box2.addWidget(portNameCurr2, 6, 1, 1, 1)
        box2.addWidget(self.portNameCurrData2, 6, 2, 1, 1)
        box2.addWidget(portNameInp2, 7, 1, 1, 1)
        box2.addWidget(self.portNameInpData2, 7, 2, 1, 1)
        box2.setColumnStretch(8, 1)

        self.groupBoxR2.setLayout(box2)

        self.groupBoxR3 = QGroupBox(self.chName['3'][0])
        self.groupBoxR3.setCheckable(True)
        self.groupBoxR3.setChecked(True)
        # self.groupBoxR3.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR3.setFont(self.fontMode)
        self.groupBoxR3.setMaximumHeight(h)
        box3 = QGridLayout()

        self.pzt3 = QCheckBox("Piezo Relock")
        self.pzt3.setFont(self.fontMode)

        self.curr3 = QCheckBox("Current Relock")
        self.curr3.setFont(self.fontMode)

        freq3 = QLabel("Frequency: ")
        freq3.setFont(self.fontMode)
        self.freqDataR3 = QLabel("")
        self.freqDataR3.setFont(self.fontMode)

        trg3 = QLabel("Target: ")
        trg3.setFont(self.fontMode)
        trgData3 = QLabel(self.chName[str(3)][1] + " THz")
        trgData3.setFont(self.fontMode)

        mode3 = QLabel("Mode: ")
        mode3.setFont(self.fontMode)
        self.modeData3 = QLabel("")
        self.modeData3.setFont(self.fontMode)

        portNamePzt3 = QLabel("Port Name Piezo: ")
        portNamePzt3.setFont(self.fontMode)
        self.portNamePztData3 = QLineEdit()
        self.portNamePztData3.setFont(self.fontMode)
        self.portNamePztData3.setPlaceholderText('Dev1/ao0')

        portNameCurr3 = QLabel("Port Name Current: ")
        portNameCurr3.setFont(self.fontMode)
        self.portNameCurrData3 = QLineEdit()
        self.portNameCurrData3.setFont(self.fontMode)
        self.portNameCurrData3.setPlaceholderText('Dev1/ao0')

        portNameInp3 = QLabel("Port Name Input: ")
        portNameInp3.setFont(self.fontMode)
        self.portNameInpData3 = QLineEdit()
        self.portNameInpData3.setFont(self.fontMode)
        self.portNameInpData3.setPlaceholderText('Dev1/ai0')

        box3.setColumnStretch(0, 1)
        box3.addWidget(self.pzt3, 0, 1, 1, 1)
        box3.addWidget(self.curr3, 1, 1, 1, 1)
        box3.addWidget(freq3, 2, 1, 1, 1)
        box3.addWidget(self.freqDataR3, 2, 2, 1, 1)
        box3.addWidget(trg3, 3, 1, 1, 1)
        box3.addWidget(trgData3, 3, 2, 1, 1)
        box3.addWidget(mode3, 4, 1, 1, 1)
        box3.addWidget(self.modeData3, 4, 2, 1, 1)
        box3.addWidget(portNamePzt3, 5, 1, 1, 1)
        box3.addWidget(self.portNamePztData3, 5, 2, 1, 1)
        box3.addWidget(portNameCurr3, 6, 1, 1, 1)
        box3.addWidget(self.portNameCurrData3, 6, 2, 1, 1)
        box3.addWidget(portNameInp3, 7, 1, 1, 1)
        box3.addWidget(self.portNameInpData3, 7, 2, 1, 1)
        box3.setColumnStretch(8, 1)

        self.groupBoxR3.setLayout(box3)

        self.groupBoxR4 = QGroupBox(self.chName['4'][0])
        self.groupBoxR4.setCheckable(True)
        self.groupBoxR4.setChecked(True)
        # self.groupBoxR4.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR4.setFont(self.fontMode)
        self.groupBoxR4.setMaximumHeight(h)
        box4 = QGridLayout()

        self.pzt4 = QCheckBox("Piezo Relock")
        self.pzt4.setFont(self.fontMode)

        self.curr4 = QCheckBox("Current Relock")
        self.curr4.setFont(self.fontMode)

        freq4 = QLabel("Frequency: ")
        freq4.setFont(self.fontMode)
        self.freqDataR4 = QLabel("")
        self.freqDataR4.setFont(self.fontMode)

        trg4 = QLabel("Target: ")
        trg4.setFont(self.fontMode)
        trgData4 = QLabel(self.chName[str(4)][1] + " THz")
        trgData4.setFont(self.fontMode)

        mode4 = QLabel("Mode: ")
        mode4.setFont(self.fontMode)
        self.modeData4 = QLabel("")
        self.modeData4.setFont(self.fontMode)

        portNamePzt4 = QLabel("Port Name Piezo: ")
        portNamePzt4.setFont(self.fontMode)
        self.portNamePztData4 = QLineEdit()
        self.portNamePztData4.setFont(self.fontMode)
        self.portNamePztData4.setPlaceholderText('Dev1/ao0')

        portNameCurr4 = QLabel("Port Name Current: ")
        portNameCurr4.setFont(self.fontMode)
        self.portNameCurrData4 = QLineEdit()
        self.portNameCurrData4.setFont(self.fontMode)
        self.portNameCurrData4.setPlaceholderText('Dev1/ao0')

        portNameInp4 = QLabel("Port Name Input: ")
        portNameInp4.setFont(self.fontMode)
        self.portNameInpData4 = QLineEdit()
        self.portNameInpData4.setFont(self.fontMode)
        self.portNameInpData4.setPlaceholderText('Dev1/ai0')

        box4.setColumnStretch(0, 1)
        box4.addWidget(self.pzt4, 0, 1, 1, 1)
        box4.addWidget(self.curr4, 1, 1, 1, 1)
        box4.addWidget(freq4, 2, 1, 1, 1)
        box4.addWidget(self.freqDataR4, 2, 2, 1, 1)
        box4.addWidget(trg4, 3, 1, 1, 1)
        box4.addWidget(trgData4, 3, 2, 1, 1)
        box4.addWidget(mode4, 4, 1, 1, 1)
        box4.addWidget(self.modeData4, 4, 2, 1, 1)
        box4.addWidget(portNamePzt4, 5, 1, 1, 1)
        box4.addWidget(self.portNamePztData4, 5, 2, 1, 1)
        box4.addWidget(portNameCurr4, 6, 1, 1, 1)
        box4.addWidget(self.portNameCurrData4, 6, 2, 1, 1)
        box4.addWidget(portNameInp4, 7, 1, 1, 1)
        box4.addWidget(self.portNameInpData4, 7, 2, 1, 1)
        box4.setColumnStretch(8, 1)

        self.groupBoxR4.setLayout(box4)

        self.groupBoxR5 = QGroupBox(self.chName['5'][0])
        self.groupBoxR5.setCheckable(True)
        self.groupBoxR5.setChecked(True)
        # self.groupBoxR5.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR5.setFont(self.fontMode)
        self.groupBoxR5.setMaximumHeight(h)
        box5 = QGridLayout()

        self.pzt5 = QCheckBox("Piezo Relock")
        self.pzt5.setFont(self.fontMode)

        self.curr5 = QCheckBox("Current Relock")
        self.curr5.setFont(self.fontMode)

        freq5 = QLabel("Frequency: ")
        freq5.setFont(self.fontMode)
        self.freqDataR5 = QLabel("")
        self.freqDataR5.setFont(self.fontMode)

        trg5 = QLabel("Target: ")
        trg5.setFont(self.fontMode)
        trgData5 = QLabel(self.chName[str(5)][1] + " THz")
        trgData5.setFont(self.fontMode)

        mode5 = QLabel("Mode: ")
        mode5.setFont(self.fontMode)
        self.modeData5 = QLabel("")
        self.modeData5.setFont(self.fontMode)

        portNamePzt5 = QLabel("Port Name Piezo: ")
        portNamePzt5.setFont(self.fontMode)
        self.portNamePztData5 = QLineEdit()
        self.portNamePztData5.setFont(self.fontMode)
        self.portNamePztData5.setPlaceholderText('Dev1/ao0')

        portNameCurr5 = QLabel("Port Name Current: ")
        portNameCurr5.setFont(self.fontMode)
        self.portNameCurrData5 = QLineEdit()
        self.portNameCurrData5.setFont(self.fontMode)
        self.portNameCurrData5.setPlaceholderText('Dev1/ao0')

        portNameInp5 = QLabel("Port Name Input: ")
        portNameInp5.setFont(self.fontMode)
        self.portNameInpData5 = QLineEdit()
        self.portNameInpData5.setFont(self.fontMode)
        self.portNameInpData5.setPlaceholderText('Dev1/ai0')

        box5.setColumnStretch(0, 1)
        box5.addWidget(self.pzt5, 0, 1, 1, 1)
        box5.addWidget(self.curr5, 1, 1, 1, 1)
        box5.addWidget(freq5, 2, 1, 1, 1)
        box5.addWidget(self.freqDataR5, 2, 2, 1, 1)
        box5.addWidget(trg5, 3, 1, 1, 1)
        box5.addWidget(trgData5, 3, 2, 1, 1)
        box5.addWidget(mode5, 4, 1, 1, 1)
        box5.addWidget(self.modeData5, 4, 2, 1, 1)
        box5.addWidget(portNamePzt5, 5, 1, 1, 1)
        box5.addWidget(self.portNamePztData5, 5, 2, 1, 1)
        box5.addWidget(portNameCurr5, 6, 1, 1, 1)
        box5.addWidget(self.portNameCurrData5, 6, 2, 1, 1)
        box5.addWidget(portNameInp5, 7, 1, 1, 1)
        box5.addWidget(self.portNameInpData5, 7, 2, 1, 1)
        box5.setColumnStretch(8, 1)

        self.groupBoxR5.setLayout(box5)

        self.groupBoxR6 = QGroupBox(self.chName['6'][0])
        self.groupBoxR6.setCheckable(True)
        self.groupBoxR6.setChecked(True)
        # self.groupBoxR6.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR6.setFont(self.fontMode)
        self.groupBoxR6.setMaximumHeight(h)
        box6 = QGridLayout()

        self.pzt6 = QCheckBox("Piezo Relock")
        self.pzt6.setFont(self.fontMode)

        self.curr6 = QCheckBox("Current Relock")
        self.curr6.setFont(self.fontMode)

        freq6 = QLabel("Frequency: ")
        freq6.setFont(self.fontMode)
        self.freqDataR6 = QLabel("")
        self.freqDataR6.setFont(self.fontMode)

        trg6 = QLabel("Target: ")
        trg6.setFont(self.fontMode)
        trgData6 = QLabel(self.chName[str(6)][1] + " THz")
        trgData6.setFont(self.fontMode)

        mode6 = QLabel("Mode: ")
        mode6.setFont(self.fontMode)
        self.modeData6 = QLabel("")
        self.modeData6.setFont(self.fontMode)

        portNamePzt6 = QLabel("Port Name Piezo: ")
        portNamePzt6.setFont(self.fontMode)
        self.portNamePztData6 = QLineEdit()
        self.portNamePztData6.setFont(self.fontMode)
        self.portNamePztData6.setPlaceholderText('Dev1/ao0')

        portNameCurr6 = QLabel("Port Name Current: ")
        portNameCurr6.setFont(self.fontMode)
        self.portNameCurrData6 = QLineEdit()
        self.portNameCurrData6.setFont(self.fontMode)
        self.portNameCurrData6.setPlaceholderText('Dev1/ao0')

        portNameInp6 = QLabel("Port Name Input: ")
        portNameInp6.setFont(self.fontMode)
        self.portNameInpData6 = QLineEdit()
        self.portNameInpData6.setFont(self.fontMode)
        self.portNameInpData6.setPlaceholderText('Dev1/ai0')

        box6.setColumnStretch(0, 1)
        box6.addWidget(self.pzt6, 0, 1, 1, 1)
        box6.addWidget(self.curr6, 1, 1, 1, 1)
        box6.addWidget(freq6, 2, 1, 1, 1)
        box6.addWidget(self.freqDataR6, 2, 2, 1, 1)
        box6.addWidget(trg6, 3, 1, 1, 1)
        box6.addWidget(trgData6, 3, 2, 1, 1)
        box6.addWidget(mode6, 4, 1, 1, 1)
        box6.addWidget(self.modeData6, 4, 2, 1, 1)
        box6.addWidget(portNamePzt6, 5, 1, 1, 1)
        box6.addWidget(self.portNamePztData6, 5, 2, 1, 1)
        box6.addWidget(portNameCurr6, 6, 1, 1, 1)
        box6.addWidget(self.portNameCurrData6, 6, 2, 1, 1)
        box6.addWidget(portNameInp6, 7, 1, 1, 1)
        box6.addWidget(self.portNameInpData6, 7, 2, 1, 1)
        box6.setColumnStretch(8, 1)

        self.groupBoxR6.setLayout(box6)

        self.groupBoxR7 = QGroupBox(self.chName['7'][0])
        self.groupBoxR7.setCheckable(True)
        self.groupBoxR7.setChecked(True)
        # self.groupBoxR7.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR7.setFont(self.fontMode)
        self.groupBoxR7.setMaximumHeight(h)
        box7 = QGridLayout()

        self.pzt7 = QCheckBox("Piezo Relock")
        self.pzt7.setFont(self.fontMode)

        self.curr7 = QCheckBox("Current Relock")
        self.curr7.setFont(self.fontMode)

        freq7 = QLabel("Frequency: ")
        freq7.setFont(self.fontMode)
        self.freqDataR7 = QLabel("")
        self.freqDataR7.setFont(self.fontMode)

        trg7 = QLabel("Target: ")
        trg7.setFont(self.fontMode)
        trgData7 = QLabel(self.chName[str(7)][1] + " THz")
        trgData7.setFont(self.fontMode)

        mode7 = QLabel("Mode: ")
        mode7.setFont(self.fontMode)
        self.modeData7 = QLabel("")
        self.modeData7.setFont(self.fontMode)

        portNamePzt7 = QLabel("Port Name Piezo: ")
        portNamePzt7.setFont(self.fontMode)
        self.portNamePztData7 = QLineEdit()
        self.portNamePztData7.setFont(self.fontMode)
        self.portNamePztData7.setPlaceholderText('Dev1/ao0')

        portNameCurr7 = QLabel("Port Name Current: ")
        portNameCurr7.setFont(self.fontMode)
        self.portNameCurrData7 = QLineEdit()
        self.portNameCurrData7.setFont(self.fontMode)
        self.portNameCurrData7.setPlaceholderText('Dev1/ao0')

        portNameInp7 = QLabel("Port Name Input: ")
        portNameInp7.setFont(self.fontMode)
        self.portNameInpData7 = QLineEdit()
        self.portNameInpData7.setFont(self.fontMode)
        self.portNameInpData7.setPlaceholderText('Dev1/ai0')

        box7.setColumnStretch(0, 1)
        box7.addWidget(self.pzt7, 0, 1, 1, 1)
        box7.addWidget(self.curr7, 1, 1, 1, 1)
        box7.addWidget(freq7, 2, 1, 1, 1)
        box7.addWidget(self.freqDataR7, 2, 2, 1, 1)
        box7.addWidget(trg7, 3, 1, 1, 1)
        box7.addWidget(trgData7, 3, 2, 1, 1)
        box7.addWidget(mode7, 4, 1, 1, 1)
        box7.addWidget(self.modeData7, 4, 2, 1, 1)
        box7.addWidget(portNamePzt7, 5, 1, 1, 1)
        box7.addWidget(self.portNamePztData7, 5, 2, 1, 1)
        box7.addWidget(portNameCurr7, 6, 1, 1, 1)
        box7.addWidget(self.portNameCurrData7, 6, 2, 1, 1)
        box7.addWidget(portNameInp7, 7, 1, 1, 1)
        box7.addWidget(self.portNameInpData7, 7, 2, 1, 1)
        box7.setColumnStretch(8, 1)

        self.groupBoxR7.setLayout(box7)

        self.groupBoxR8 = QGroupBox(self.chName['8'][0])
        self.groupBoxR8.setCheckable(True)
        self.groupBoxR8.setChecked(True)
        # self.groupBoxR8.setStyleSheet("color: rgb(0, 0, 255)")
        self.groupBoxR8.setFont(self.fontMode)
        self.groupBoxR8.setMaximumHeight(h)
        box8 = QGridLayout()

        self.pzt8 = QCheckBox("Piezo Relock")
        self.pzt8.setFont(self.fontMode)

        self.curr8 = QCheckBox("Current Relock")
        self.curr8.setFont(self.fontMode)

        freq8 = QLabel("Frequency: ")
        freq8.setFont(self.fontMode)
        self.freqDataR8 = QLabel("")
        self.freqDataR8.setFont(self.fontMode)

        trg8 = QLabel("Target: ")
        trg8.setFont(self.fontMode)
        trgData8 = QLabel(self.chName[str(8)][1] + " THz")
        trgData8.setFont(self.fontMode)

        mode8 = QLabel("Mode: ")
        mode8.setFont(self.fontMode)
        self.modeData8 = QLabel("")
        self.modeData8.setFont(self.fontMode)

        portNamePzt8 = QLabel("Port Name Piezo: ")
        portNamePzt8.setFont(self.fontMode)
        self.portNamePztData8 = QLineEdit()
        self.portNamePztData8.setFont(self.fontMode)
        self.portNamePztData8.setPlaceholderText('Dev1/ao0')

        portNameCurr8 = QLabel("Port Name Current: ")
        portNameCurr8.setFont(self.fontMode)
        self.portNameCurrData8 = QLineEdit()
        self.portNameCurrData8.setFont(self.fontMode)
        self.portNameCurrData8.setPlaceholderText('Dev1/ao0')

        portNameInp8 = QLabel("Port Name Input: ")
        portNameInp8.setFont(self.fontMode)
        self.portNameInpData8 = QLineEdit()
        self.portNameInpData8.setFont(self.fontMode)
        self.portNameInpData8.setPlaceholderText('Dev1/ai0')

        box8.setColumnStretch(0, 1)
        box8.addWidget(self.pzt8, 0, 1, 1, 1)
        box8.addWidget(self.curr8, 1, 1, 1, 1)
        box8.addWidget(freq8, 2, 1, 1, 1)
        box8.addWidget(self.freqDataR8, 2, 2, 1, 1)
        box8.addWidget(trg8, 3, 1, 1, 1)
        box8.addWidget(trgData8, 3, 2, 1, 1)
        box8.addWidget(mode8, 4, 1, 1, 1)
        box8.addWidget(self.modeData8, 4, 2, 1, 1)
        box8.addWidget(portNamePzt8, 5, 1, 1, 1)
        box8.addWidget(self.portNamePztData8, 5, 2, 1, 1)
        box8.addWidget(portNameCurr8, 6, 1, 1, 1)
        box8.addWidget(self.portNameCurrData8, 6, 2, 1, 1)
        box8.addWidget(portNameInp8, 7, 1, 1, 1)
        box8.addWidget(self.portNameInpData8, 7, 2, 1, 1)
        box8.setColumnStretch(8, 1)

        self.groupBoxR8.setLayout(box8)
        
        self.relockInfo.addWidget(self.groupBoxR1, 0, 0, 1, 1)
        self.relockInfo.addWidget(self.groupBoxR2, 0, 1, 1, 1)
        self.relockInfo.addWidget(self.groupBoxR3, 0, 3, 1, 1)
        self.relockInfo.addWidget(self.groupBoxR4, 0, 4, 1, 1)
        self.relockInfo.addWidget(self.groupBoxR5, 1, 0, 1, 1)
        self.relockInfo.addWidget(self.groupBoxR6, 1, 1, 1, 1)
        self.relockInfo.addWidget(self.groupBoxR7, 1, 3, 1, 1)
        self.relockInfo.addWidget(self.groupBoxR8, 1, 4, 1, 1)
        self.relockInfo.setRowStretch(2, 1)
    
    def getInitialValues(self):

        try:

            if self.currTab == 0:
                
                self.ch = self.wlm.getSwitcherChannel()
                if self.ch < 1 or self.ch > 8:
                    self.ch = 1
                self.wave = self.wlm.getWavelength(self.ch)
                self.freq = self.wlm.getFrequency(self.ch)
                self.spec = self.wlm.spectrum(self.ch)
            self.expoUp = self.wlm.getExposure(self.ch, 1)      # interferometer
            self.expoDown = self.wlm.getExposure(self.ch, 2)    # wide interferometer
            self.expoAut = self.wlm.getExposureMode(self.ch)
        except AttributeError:
            pass
        
        self.plotPattern.setXRange(0, 2050, padding=0)
        self.plotPattern.setYRange(0, 4200, padding=0.02)
        # self.plotStd.setXRange(0, 2050, padding=0)

        try:
            
            if self.currTab == 0:
                
                self.waveData.setText(str(f'{self.wave:.4f}') + " nm")
                self.freqData.setText(str(f'{self.freq:.4f}') + " THz")
                self.tgfData.setText(self.chName[str(self.ch)][1] + " THz")
                self.chData.setCurrentIndex(self.ch -1)
                self.data.setData(self.spec)
            self.upData.setValue(self.expoUp)
            self.downData.setValue(self.expoDown)
            self.automaticExpo.setChecked(self.expoAut)
        except AttributeError:
            pass    

        self.currS.setMinimum(int(self.IParam[str(self.ch)][0]/self.currStep))
        self.currS.setMaximum(int(self.IParam[str(self.ch)][1]/self.currStep))

        self.currSb.setMinimum(self.IParam[str(self.ch)][0])
        self.currSb.setMaximum(self.IParam[str(self.ch)][1])

        self.piezoS.setMinimum(int(self.PztParam[str(self.ch)][0]/self.PztStep))
        self.piezoS.setMaximum(int(self.PztParam[str(self.ch)][1]/self.PztStep))

        self.piezoSb.setMinimum(self.PztParam[str(self.ch)][0])
        self.piezoSb.setMaximum(self.PztParam[str(self.ch)][1])

        if self.currTab == 0:

            port = self.chName[str(self.ch)][6]
            if port:
                self.currS.setValue(int(self.IParam[str(self.ch)][2]/self.currStep))
                self.currSb.setValue(self.IParam[str(self.ch)][2])
            port = self.chName[str(self.ch)][5]
            if port:
                self.piezoS.setValue(int(self.PztParam[str(self.ch)][2]/self.PztStep))
                self.piezoSb.setValue(self.PztParam[str(self.ch)][2])

            if self.refDataInfo[str(self.ch)][3] == 0:
                self.setRef.setStyleSheet("background-color : red")
            else:
                self.setRef.setStyleSheet("background-color : ")

            if self.chName[str(self.ch)][2] == 1:

                self.pztRelock.setChecked(True)
            else:

                self.pztRelock.setChecked(False)
            if self.chName[str(self.ch)][3] == 1:

                self.currRelock.setChecked(True)
            else:

                self.currRelock.setChecked(False)

    def changeChannel(self):

        self.ch = self.chData.currentIndex() + 1

        self.wlm.switcherChannel(self.ch)
        self.getInitialValues()

    def expoAutomatic(self):

        if self.automaticExpo.isChecked():
            
            self.expoAut = 1
            self.wlm.setExposureMode(self.ch, True)
        else:
            self.expoAut = 0
            self.wlm.setExposureMode(self.ch, False)

    def exposureUp(self):

        # change for interferometer
        self.expoUp = self.upData.value()

        self.wlm.setExposure(self.ch, 1, self.expoUp)

    def exposureDown(self):

        # change for wide interferometer
        val = self.downData.value()
        if self.expoDown > val and val == 1:
            self.expoDown = 0
            self.downData.setValue(self.expoDown)
        elif val == 1:
            self.expoDown = 2
            self.downData.setValue(self.expoDown)

        self.wlm.setExposure(self.ch, 2, self.expoDown)
    
    def pztRelockState(self):
        
        if self.pztRelock.isChecked():

            if self.currTab == 0:
                
                port = self.chName[str(self.ch)][5]
                if port:

                    self.chName[str(self.ch)][2] = 1

                    if self.ch == 1:
                        self.pzt1.setChecked(True)
                    elif self.ch == 2:
                        self.pzt2.setChecked(True)
                    elif self.ch == 3:
                        self.pzt3.setChecked(True)
                    elif self.ch == 4:
                        self.pzt4.setChecked(True)
                    elif self.ch == 5:
                        self.pzt5.setChecked(True)
                    elif self.ch == 6:
                        self.pzt6.setChecked(True)
                    elif self.ch == 7:
                        self.pzt7.setChecked(True)
                    elif self.ch == 8:
                        self.pzt8.setChecked(True)

                else:
                    self.sendMsg("assign a port for changing the piezo!")
                    self.pztRelock.setCheckState(0)         # if we want to change the state inside itself we have to use this function
        else:

            if self.currTab == 0:
                
                self.chName[str(self.ch)][2] = 0

                if self.ch == 1:
                    self.pzt1.setChecked(False)
                elif self.ch == 2:
                    self.pzt2.setChecked(False)
                elif self.ch == 3:
                    self.pzt3.setChecked(False)
                elif self.ch == 4:
                    self.pzt4.setChecked(False)
                elif self.ch == 5:
                    self.pzt5.setChecked(False)
                elif self.ch == 6:
                    self.pzt6.setChecked(False)
                elif self.ch == 7:
                    self.pzt7.setChecked(False)
                elif self.ch == 8:
                    self.pzt8.setChecked(False)
    
    def currRelockState(self):
        
        if self.currRelock.isChecked():

            if self.currTab == 0:
                
                port = self.chName[str(self.ch)][6]
                if port:

                    if self.refDataInfo[str(self.ch)][3] == 1:

                        self.chName[str(self.ch)][3] = 1

                        if self.ch == 1:
                            self.curr1.setChecked(True)
                        elif self.ch == 2:
                            self.curr2.setChecked(True)
                        elif self.ch == 3:
                            self.curr3.setChecked(True)
                        elif self.ch == 4:
                            self.curr4.setChecked(True)
                        elif self.ch == 5:
                            self.curr5.setChecked(True)
                        elif self.ch == 6:
                            self.curr6.setChecked(True)
                        elif self.ch == 7:
                            self.curr7.setChecked(True)
                        elif self.ch == 8:
                            self.curr8.setChecked(True)

                    else:
                        self.sendMsg("there is not any refrence pattern for this channel!")
                        self.currRelock.setCheckState(0)

                else:
                    self.sendMsg("assign a port for changing the current!")
                    self.currRelock.setCheckState(0)
        else:

            if self.currTab == 0:
                
                self.chName[str(self.ch)][3] = 0

                if self.ch == 1:
                    self.curr1.setChecked(False)
                elif self.ch == 2:
                    self.curr2.setChecked(False)
                elif self.ch == 3:
                    self.curr3.setChecked(False)
                elif self.ch == 4:
                    self.curr4.setChecked(False)
                elif self.ch == 5:
                    self.curr5.setChecked(False)
                elif self.ch == 6:
                    self.curr6.setChecked(False)
                elif self.ch == 7:
                    self.curr7.setChecked(False)
                elif self.ch == 8:
                    self.curr8.setChecked(False)

    def pzt1RelockStateDetails(self):

        if self.pzt1.isChecked():

            port = self.chName[str(1)][5]
            if port:
                self.chName[str(1)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt1.setCheckState(0)
        else:
            self.chName[str(1)][2] = 0

    def pzt2RelockStateDetails(self):

        if self.pzt2.isChecked():

            port = self.chName[str(2)][5]
            if port:
                self.chName[str(2)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt2.setCheckState(0)
        else:
            self.chName[str(2)][2] = 0

    def pzt3RelockStateDetails(self):

        if self.pzt3.isChecked():

            port = self.chName[str(3)][5]
            if port:
                self.chName[str(3)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt3.setCheckState(0)
        else:
            self.chName[str(3)][2] = 0

    def pzt4RelockStateDetails(self):

        if self.pzt4.isChecked():

            port = self.chName[str(4)][5]
            if port:
                self.chName[str(4)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt4.setCheckState(0)
        else:
            self.chName[str(4)][2] = 0

    def pzt5RelockStateDetails(self):

        if self.pzt5.isChecked():

            port = self.chName[str(5)][5]
            if port:
                self.chName[str(5)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt5.setCheckState(0)
        else:
            self.chName[str(5)][2] = 0
        
    def pzt6RelockStateDetails(self):

        if self.pzt6.isChecked():

            port = self.chName[str(6)][5]
            if port:
                self.chName[str(6)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt6.setCheckState(0)
        else:
            self.chName[str(6)][2] = 0

    def pzt7RelockStateDetails(self):

        if self.pzt7.isChecked():

            port = self.chName[str(7)][5]
            if port:
                self.chName[str(7)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt7.setCheckState(0)
        else:
            self.chName[str(7)][2] = 0

    def pzt8RelockStateDetails(self):

        if self.pzt8.isChecked():

            port = self.chName[str(8)][5]
            if port:
                self.chName[str(8)][2] = 1
            else:
                self.sendMsg("assign a port for changing the piezo!")
                self.pzt8.setCheckState(0)
        else:
            self.chName[str(8)][2] = 0

    def curr1RelockStateDetails(self):

        if self.curr1.isChecked():

            port = self.chName[str(1)][6]
            if port:

                if self.refDataInfo[str(1)][3] == 1:
                    self.chName[str(1)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr1.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr1.setCheckState(0)
        else:
            self.chName[str(1)][3] = 0

    def curr2RelockStateDetails(self):

        if self.curr2.isChecked():

            port = self.chName[str(2)][6]
            if port:

                if self.refDataInfo[str(2)][3] == 1:
                    self.chName[str(2)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr2.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr2.setCheckState(0)
        else:
            self.chName[str(2)][3] = 0

    def curr3RelockStateDetails(self):

        if self.curr3.isChecked():

            port = self.chName[str(3)][6]
            if port:

                if self.refDataInfo[str(3)][3] == 1:
                    self.chName[str(3)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr3.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr3.setCheckState(0)
        else:
            self.chName[str(3)][3] = 0

    def curr4RelockStateDetails(self):

        if self.curr4.isChecked():

            port = self.chName[str(4)][6]
            if port:

                if self.refDataInfo[str(4)][3] == 1:
                    self.chName[str(4)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr4.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr4.setCheckState(0)
        else:
            self.chName[str(4)][3] = 0

    def curr5RelockStateDetails(self):

        if self.curr5.isChecked():

            port = self.chName[str(5)][6]
            if port:

                if self.refDataInfo[str(5)][3] == 1:
                    self.chName[str(5)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr5.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr5.setCheckState(0)
        else:
            self.chName[str(5)][3] = 0
        
    def curr6RelockStateDetails(self):

        if self.curr6.isChecked():

            port = self.chName[str(6)][6]
            if port:

                if self.refDataInfo[str(6)][3] == 1:
                    self.chName[str(6)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr6.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr6.setCheckState(0)
        else:
            self.chName[str(6)][3] = 0

    def curr7RelockStateDetails(self):

        if self.curr7.isChecked():

            port = self.chName[str(7)][6]
            if port:

                if self.refDataInfo[str(7)][3] == 1:
                    self.chName[str(7)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr7.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr7.setCheckState(0)
        else:
            self.chName[str(7)][3] = 0

    def curr8RelockStateDetails(self):

        if self.curr8.isChecked():

            port = self.chName[str(8)][6]
            if port:

                if self.refDataInfo[str(8)][3] == 1:
                    self.chName[str(8)][3] = 1
                else:
                    self.sendMsg("there is not any refrence pattern for this channel!")
                    self.curr8.setCheckState(0)
            else:
                self.sendMsg("assign a port for changing the current!")
                self.curr8.setCheckState(0)
        else:
            self.chName[str(8)][3] = 0

    def sendMsg(self, text):

        winIcon = QIcon()
        winIcon.addPixmap(QPixmap(path + "icon\\ealert.png"), QIcon.Normal, QIcon.Off)
        icon = QMessageBox.Information
        title = "Alert"
        type = QMessageBox.Ok
        self.notification(winIcon, icon, title, text, type)

    def portChangePzt1(self):

        self.chName[str(1)][5] = self.portNamePztData1.text()
    
    def portChangePzt2(self):

        self.chName[str(2)][5] = self.portNamePztData2.text()

    def portChangePzt3(self):

        self.chName[str(3)][5] = self.portNamePztData3.text()
    
    def portChangePzt4(self):

        self.chName[str(4)][5] = self.portNamePztData4.text()
        
    def portChangePzt5(self):

        self.chName[str(5)][5] = self.portNamePztData5.text()

    def portChangePzt6(self):

        self.chName[str(6)][5] = self.portNamePztData6.text()
    
    def portChangePzt7(self):

        self.chName[str(7)][5] = self.portNamePztData7.text()

    def portChangePzt8(self):

        self.chName[str(8)][5] = self.portNamePztData8.text()

    def portChangeCurr1(self):

        self.chName[str(1)][6] = self.portNameCurrData1.text()
    
    def portChangeCurr2(self):

        self.chName[str(2)][6] = self.portNameCurrData2.text()
        
    def portChangeCurr3(self):

        self.chName[str(3)][6] = self.portNameCurrData3.text()
        
    def portChangeCurr4(self):

        self.chName[str(4)][6] = self.portNameCurrData4.text()
        
    def portChangeCurr5(self):

        self.chName[str(5)][6] = self.portNameCurrData5.text()
        
    def portChangeCurr6(self):

        self.chName[str(6)][6] = self.portNameCurrData6.text()
        
    def portChangeCurr7(self):

        self.chName[str(7)][6] = self.portNameCurrData7.text()
        
    def portChangeCurr8(self):

        self.chName[str(8)][6] = self.portNameCurrData8.text()

    def portChangeInp1(self):

        self.chName[str(1)][7] = self.portNameInpData1.text()

    def portChangeInp2(self):

        self.chName[str(2)][7] = self.portNameInpData2.text()
        
    def portChangeInp3(self):

        self.chName[str(3)][7] = self.portNameInpData3.text()
        
    def portChangeInp4(self):

        self.chName[str(4)][7] = self.portNameInpData4.text()
        
    def portChangeInp5(self):

        self.chName[str(5)][7] = self.portNameInpData5.text()
        
    def portChangeInp6(self):

        self.chName[str(6)][7] = self.portNameInpData6.text()
        
    def portChangeInp7(self):

        self.chName[str(7)][7] = self.portNameInpData7.text()
        
    def portChangeInp8(self):

        self.chName[str(8)][7] = self.portNameInpData8.text()

    def stRelockState(self):

        self.monitor = True
        if self.stRelock.text() == "Start Relocking":

            self.relockMode = True
            self.stRelock.setText("Stop Relocking")
            self.stRelock.setStyleSheet("background-color : red")
            # self.relT0 = time.time()   # sec
        else:
            self.corr = False
            self.relockMode = False
            self.infoMsg.setText(" ")
            self.stRelock.setText("Start Relocking")
            self.stRelock.setStyleSheet("background-color : ")

    def setReference(self):

        self.refData[str(self.ch)] = self.spec
        
        self.saveFiles(str(self.ch))
        peaksIndRef, peaksHRef = find_peaks(self.refData[str(self.ch)], height = self.height_thr, distance = self.distance_thr)
        self.refDataInfo[str(self.ch)][0] = len(peaksIndRef)
        self.refDataInfo[str(self.ch)][3] == 1
        self.setRef.setStyleSheet("background-color : ")

    def showHighFinesse(self):

        if self.highFinesse.isChecked():
            
            self.wlm.run('show')
        else:
            
            self.wlm.run('hide')

    def operation(self):

        if self.start.text() == "Start":

            err = self.wlm.measurement(self.measurement)
            if err == ResERR_WlmMissing:
                return

            self.start.setText("Stop")
            self.infoMsg.setText(" ")

            self.upData.setDisabled(False)
            self.downData.setDisabled(False)
            self.automaticExpo.setDisabled(False)

            self.pztRelock.setEnabled(True)
            self.currRelock.setEnabled(True)
            self.stRelock.setEnabled(True)

            self.setRef.setEnabled(True)
        
            self.currS.setDisabled(False)
            self.currSb.setDisabled(False)
            self.piezoS.setDisabled(False)
            self.piezoSb.setDisabled(False)

            if self.currTab == 0: 
                self.getInitialValues()
            self.updT0 = time.time()   # sec
            self.relT0 = time.time()   # sec
        else:
            
            self.start.setText("Start")
            self.infoMsg.setText("Run...")
            if self.relockMode:
                self.stRelockState()
            self.mode = True

            self.upData.setDisabled(True)
            self.downData.setDisabled(True)
            self.automaticExpo.setDisabled(True)

            self.pztRelock.setEnabled(False)
            self.currRelock.setEnabled(False)
            self.stRelock.setEnabled(False)
            
            self.setRef.setEnabled(False)
        
            self.currS.setDisabled(True)
            self.currSb.setDisabled(True)
            self.piezoS.setDisabled(True)
            self.piezoSb.setDisabled(True)

            self.wlm.measurement(self.stop)

    def lc_current(self):

        val = self.currS.value()*self.currStep

        if not self.lcState:
            self.lcState = True
            self.currSb.setValue(val)
        else:
            self.lcState = False

        port = self.chName[str(self.ch)][6]
        if port:
        
            if self.start.text() != "Start":

                err = self.lc.setOutput(port, val)
                if err == False:
                    self.sendMsg("assign a correct port for changing the current!")
                time.sleep(0.01)

                if val > self.IParam[str(self.ch)][2]:

                    self.IParam[str(self.ch)][3] = '+'
                else:

                    self.IParam[str(self.ch)][3] = '-'

                self.IParam[str(self.ch)][2] = val
        # else:
        #     self.sendMsg("assign a port for changing the current!")

    def lc_currentB(self):

        val = self.currSb.value()

        if not self.lcState:
            self.lcState = True
            self.currS.setValue(int(val/self.currStep))
        else:
            self.lcState = False

    def lc_piezo(self):

        val = self.piezoS.value()*self.PztStep

        if not self.lcState:
            self.lcState = True
            self.piezoSb.setValue(val)
        else:
            self.lcState = False

        port = self.chName[str(self.ch)][5]
        if port:

            if self.start.text() != "Start":

                err = self.lc.setOutput(port, val)
                if err == False:
                    self.sendMsg("assign a correct port for changing the piezo!")
                time.sleep(0.01)

                if val > self.PztParam[str(self.ch)][2]:

                    self.PztParam[str(self.ch)][3] = '+'
                else:

                    self.PztParam[str(self.ch)][3] = '-'

                self.PztParam[str(self.ch)][2] = val
        # else:
        #     self.sendMsg("assign a port for changing the piezo!")

    def lc_piezoB(self):

        val = self.piezoSb.value()

        if not self.lcState:
            self.lcState = True
            self.piezoS.setValue(int(val/self.PztStep))
        else:
            self.lcState = False

    def setPlotRange(self):

        # a = '{}'.format((self.plotPattern.getPlotItem().getAxis('left')).range)
        # print(a[8:14])

        # if a[8:14] != '4284.0':
            
        self.range += 1

        self.plotPattern.getPlotItem().getAxis('bottom').setTickSpacing()
        self.plotPattern.getPlotItem().getAxis('left').setTickSpacing()

        if self.range == 2:

            self.range = 0
            self.plotPattern.setYRange(0, 4200, padding=0.02)
            self.plotPattern.setXRange(0, 2050, padding=0)

            self.plotPattern.getPlotItem().getAxis('bottom').setTickSpacing(50, 2050)
            self.plotPattern.getPlotItem().getAxis('left').setTickSpacing(200, 4200)
            
            # self.plotStd.setXRange(0, 2050, padding=0)

    def setOverview(self):

        self.currOverTab = self.initialTabs.currentIndex()
        self.mode = True
        time.sleep(0.1)
        if self.currOverTab == 1:

            self.setUpd()

            if not self.updMode and not self.corr:
                self.updMode = True
                threadUpd = threading.Thread(target=self.updAllInfo)
                threadUpd.daemon = True
                threadUpd.start()
        else:

            self.saveUpd()
            if not self.updMode and not self.corr:
                self.changeChannel()

    def setUpd(self):

        self.pzt1.setChecked(self.chName['1'][2])
        self.curr1.setChecked(self.chName['1'][3])
        self.groupBoxR1.setChecked(self.chName['1'][4])
        self.portNamePztData1.setText(self.chName['1'][5])
        self.portNameCurrData1.setText(self.chName['1'][6])
        self.portNameInpData1.setText(self.chName['1'][7])

        self.pzt2.setChecked(self.chName['2'][2])
        self.curr2.setChecked(self.chName['2'][3])
        self.groupBoxR2.setChecked(self.chName['2'][4])
        self.portNamePztData2.setText(self.chName['2'][5])
        self.portNameCurrData2.setText(self.chName['2'][6])
        self.portNameInpData2.setText(self.chName['2'][7])
        
        self.pzt3.setChecked(self.chName['3'][2])
        self.curr3.setChecked(self.chName['3'][3])
        self.groupBoxR3.setChecked(self.chName['3'][4])
        self.portNamePztData3.setText(self.chName['3'][5])
        self.portNameCurrData3.setText(self.chName['3'][6])
        self.portNameInpData3.setText(self.chName['3'][7])
        
        self.pzt4.setChecked(self.chName['4'][2])
        self.curr4.setChecked(self.chName['4'][3])
        self.groupBoxR4.setChecked(self.chName['4'][4])
        self.portNamePztData4.setText(self.chName['4'][5])
        self.portNameCurrData4.setText(self.chName['4'][6])
        self.portNameInpData4.setText(self.chName['4'][7])
        
        self.pzt5.setChecked(self.chName['5'][2])
        self.curr5.setChecked(self.chName['5'][3])
        self.groupBoxR5.setChecked(self.chName['5'][4])
        self.portNamePztData5.setText(self.chName['5'][5])
        self.portNameCurrData5.setText(self.chName['5'][6])
        self.portNameInpData5.setText(self.chName['5'][7])
        
        self.pzt6.setChecked(self.chName['6'][2])
        self.curr6.setChecked(self.chName['6'][3])
        self.groupBoxR6.setChecked(self.chName['6'][4])
        self.portNamePztData6.setText(self.chName['6'][5])
        self.portNameCurrData6.setText(self.chName['6'][6])
        self.portNameInpData6.setText(self.chName['6'][7])
        
        self.pzt7.setChecked(self.chName['7'][2])
        self.curr7.setChecked(self.chName['7'][3])
        self.groupBoxR7.setChecked(self.chName['7'][4])
        self.portNamePztData7.setText(self.chName['7'][5])
        self.portNameCurrData7.setText(self.chName['7'][6])
        self.portNameInpData7.setText(self.chName['7'][7])
        
        self.pzt8.setChecked(self.chName['8'][2])
        self.curr8.setChecked(self.chName['8'][3])
        self.groupBoxR8.setChecked(self.chName['8'][4])
        self.portNamePztData8.setText(self.chName['8'][5])
        self.portNameCurrData8.setText(self.chName['8'][6])
        self.portNameInpData8.setText(self.chName['8'][7])

    def saveUpd(self):

        if self.groupBoxR1.isChecked():
            self.chName['1'][4] = 1
        else:
            self.chName['1'][4] = 0

        if self.groupBoxR2.isChecked():
            self.chName['2'][4] = 1
        else:
            self.chName['2'][4] = 0

        if self.groupBoxR3.isChecked():
            self.chName['3'][4] = 1
        else:
            self.chName['3'][4] = 0

        if self.groupBoxR4.isChecked():
            self.chName['4'][4] = 1
        else:
            self.chName['4'][4] = 0

        if self.groupBoxR5.isChecked():
            self.chName['5'][4] = 1
        else:
            self.chName['5'][4] = 0

        if self.groupBoxR6.isChecked():
            self.chName['6'][4] = 1
        else:
            self.chName['6'][4] = 0

        if self.groupBoxR7.isChecked():
            self.chName['7'][4] = 1
        else:
            self.chName['7'][4] = 0

        if self.groupBoxR8.isChecked():
            self.chName['8'][4] = 1
        else:
            self.chName['8'][4] = 0

    def setSwitchMode(self):
        
        self.currTab = self.tabs.currentIndex()
        self.mode = True
        time.sleep(0.1)
        if self.currTab == 1:

            self.tabs.setFixedHeight(300)
            self.infoMsg.setText("Switch Mode")

            self.setRef.setEnabled(False)
            self.currS.setDisabled(True)
            self.currSb.setDisabled(True)
            self.piezoS.setDisabled(True)
            self.piezoSb.setDisabled(True)
        else:

            self.tabs.setFixedHeight(200)
            self.data1.clear()
            self.data2.clear()
            self.data3.clear()
            self.data4.clear()
            self.data5.clear()
            self.data6.clear()
            self.data7.clear()
            self.data8.clear()
            self.setRef.setEnabled(True)

            if not self.corr:
            
                self.infoMsg.setText(" ")
                self.currS.setDisabled(False)
                self.currSb.setDisabled(False)
                self.piezoS.setDisabled(False)
                self.piezoSb.setDisabled(False)
                self.changeChannel()

    def readFile(self, ch):

        try:
                
            with open(self.fname[ch], 'r') as f:

                for line in f.readlines():

                    self.refData[ch].append(float(line))
                f.close()
                return 1
        except IOError:
            return 0

    def saveFiles(self, ch):

        try:
            
            with open(self.fname[ch], 'w') as f:

                f.write('\n'.join(str(item) for item in self.refData[ch]))
                f.close()
        except IOError:

            winIcon = QIcon()
            winIcon.addPixmap(QPixmap(path + "icon\\ealert.png"), QIcon.Normal, QIcon.Off)
            icon = QMessageBox.Information
            title = "Alert"
            text = "could not register refrence pattern for ch " + ch + ", try again!"
            type = QMessageBox.Ok
            self.notification(winIcon, icon, title, text, type)

    def get_info(self):

        try:

            if self.ch != self.wlm.getSwitcherChannel() and not self.corr:
                self.getInitialValues()
                return
            self.wave = self.wlm.getWavelength(self.ch)
            if self.wave == ErrWlmMissing or self.freq == ErrWlmMissing:
                self.operation()
                return
            if self.wave == -4 and not self.automaticExpo.isChecked() and self.corr:
            
                self.wlm.setExposureMode(self.ch, True)
                time.sleep(0.1)
                self.wlm.setExposureMode(self.ch, False)
                self.wave = self.wlm.getWavelength(self.ch)
            self.freq = self.wlm.getFrequency(self.ch)
            self.spec = self.wlm.spectrum(self.ch)
            expo = self.wlm.getExposure(self.ch, 1)      # interferometer
            if self.expoUp != expo:
                self.expoUp = expo
                self.expoChang = True
            expo = self.wlm.getExposure(self.ch, 2)    # wide interferometer
            if self.expoDown != expo:
                self.expoDown = expo
                self.expoChang = True
            expo = self.wlm.getExposureMode(self.ch)
            if self.expoAut != expo:
                self.expoAut = expo
                self.expoChang = True
            
            if self.corr:

                diff = abs(float(self.chName[str(self.ch)][1]) - self.freq)
                if diff < 50:

                    self.freq_diff = diff
                    if self.refDataInfo[str(self.ch)][3] == 1:

                        peaksIndSpec, peaksHSpec = find_peaks(self.spec, height = self.height_thr, distance = self.distance_thr)
                        self.noPeaksSpec = len(peaksIndSpec)
                        self.noPeaks_diff = abs(self.noPeaksSpec - self.refDataInfo[str(self.ch)][0])
                elif self.freq == -4:
                    self.freq_diff = -1
                    self.noPeaks_diff = -1
        except AttributeError:
            pass

    def getSpecificInfo(self, ch):

        self.freqCh = self.wlm.getFrequency(ch)
        if self.freqCh == -4 and not self.automaticExpo.isChecked():
            
            self.wlm.setExposureMode(ch, True)
            time.sleep(0.1)
            self.wlm.setExposureMode(ch, False)
            self.freqCh = self.wlm.getFrequency(ch)

        self.diffCh = abs(float(self.chName[str(ch)][1]) - self.freqCh)
        if self.diffCh < 50:
            
            if self.refDataInfo[str(self.ch)][3] == 1:

                spec = self.wlm.spectrum(ch)
                peaksIndSpec, peaksHSpec = find_peaks(spec, height = self.height_thr, distance = self.distance_thr)
                self.noPeaksSpec = len(peaksIndSpec)
                self.noPeaks_diff_ch = abs(self.noPeaksSpec - self.refDataInfo[str(ch)][0])
            else:
                self.noPeaks_diff_ch = -1
        else:
            self.diffCh = -1
            self.noPeaks_diff_ch = -2
    
    def updAllInfo(self):

        msg = ""
        if self.groupBoxR1.isChecked():

            self.getSpecificInfo(1)
            self.freqDataR1.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_921_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(1)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR1.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR1.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData1.setText('Multi')
                self.modeData1.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData1.setText('No Ref')
                self.modeData1.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData1.setText('Single')
                self.modeData1.setStyleSheet("color: rgb(0, 0, 255)")
        
        time.sleep(0.01)

        if self.groupBoxR2.isChecked():

            self.getSpecificInfo(2)
            self.freqDataR2.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_0_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(2)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR2.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR2.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData2.setText('Multi')
                self.modeData2.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData2.setText('No Ref')
                self.modeData2.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData2.setText('Single')
                self.modeData2.setStyleSheet("color: rgb(0, 0, 255)")
        
        time.sleep(0.01)

        if self.groupBoxR3.isChecked():

            self.getSpecificInfo(3)
            self.freqDataR3.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_0_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(3)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR3.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR3.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData3.setText('Multi')
                self.modeData3.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData3.setText('No Ref')
                self.modeData3.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData3.setText('Single')
                self.modeData3.setStyleSheet("color: rgb(0, 0, 255)")

        time.sleep(0.01)
        
        if self.groupBoxR4.isChecked():

            self.getSpecificInfo(4)
            self.freqDataR4.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_813_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(4)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR4.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR4.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData4.setText('Multi')
                self.modeData4.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData4.setText('No Ref')
                self.modeData4.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData4.setText('Single')
                self.modeData4.setStyleSheet("color: rgb(0, 0, 255)")

        time.sleep(0.01)
        
        if self.groupBoxR5.isChecked():

            self.getSpecificInfo(5)
            self.freqDataR5.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_679_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(5)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR5.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR5.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData5.setText('Multi')
                self.modeData5.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData5.setText('No Ref')
                self.modeData5.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData5.setText('Single')
                self.modeData5.setStyleSheet("color: rgb(0, 0, 255)")

        time.sleep(0.01)
        
        if self.groupBoxR6.isChecked():

            self.getSpecificInfo(6)
            self.freqDataR6.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_707_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(6)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR6.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR6.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData6.setText('Multi')
                self.modeData6.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData6.setText('No Ref')
                self.modeData6.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData6.setText('Single')
                self.modeData6.setStyleSheet("color: rgb(0, 0, 255)")

        time.sleep(0.01)
        
        if self.groupBoxR7.isChecked():

            self.getSpecificInfo(7)
            self.freqDataR7.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_689_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(7)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR7.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR7.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData7.setText('Multi')
                self.modeData7.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData7.setText('No Ref')
                self.modeData7.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData7.setText('Single')
                self.modeData7.setStyleSheet("color: rgb(0, 0, 255)")

        time.sleep(0.01)
        
        if self.groupBoxR8.isChecked():

            self.getSpecificInfo(8)
            self.freqDataR8.setText(str(f'{self.freqCh:.4f}') + " THz")
            msg += "wm_698_f," + str(f'{self.freqCh:.4f}') + ";"
            
            if self.diffCh < (self.refDataInfo[str(8)][1] + 0.0001) and self.diffCh >= 0:
                self.freqDataR8.setStyleSheet("color: rgb(0, 0, 255)")
            else:
                self.freqDataR8.setStyleSheet("color: rgb(255, 0, 0)")
            
            if self.noPeaks_diff_ch > self.noP_diff_thr:
                self.modeData8.setText('Multi')
                self.modeData8.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -1:
                self.modeData8.setText('No Ref')
                self.modeData8.setStyleSheet("color: rgb(255, 0, 0)")
            elif self.noPeaks_diff_ch == -2:
                pass
            else:
                self.modeData8.setText('Single')
                self.modeData8.setStyleSheet("color: rgb(0, 0, 255)")

        time.sleep(0.01)
        if self.currOverTab == 0:
            self.changeChannel()

        try:
            if msg != "":
                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.connect(self.ADDR)
                client.send(msg.encode(self.FORMAT))
                client.close()
        except OSError:
            pass

        self.updMode = False
        self.updT0 = time.time()
        self.infoMsg.setText(" ")

    def updSwitchMode(self):

        if self.groupBox1.isChecked():

            self.waveData1.setText(str(f'{self.wave1:.4f}') + " nm")
            self.freqData1.setText(str(f'{self.freq1:.4f}') + " THz")
            
            if self.show1.isChecked():
                
                self.data1.setData(self.spec1)
            else:
                self.data1.clear()
                self.data1.setData()
        else:
            self.data1.clear()
            self.data1.setData()

        if self.groupBox2.isChecked():
            
            self.waveData2.setText(str(f'{self.wave2:.4f}') + " nm")
            self.freqData2.setText(str(f'{self.freq2:.4f}') + " THz")

            if self.show2.isChecked():
                
                self.data2.setData(self.spec2)
            else:
                self.data2.clear()
                self.data2.setData()
        else:
            self.data2.clear()
            self.data2.setData()
            
        if self.groupBox3.isChecked():
            
            self.waveData3.setText(str(f'{self.wave3:.4f}') + " nm")
            self.freqData3.setText(str(f'{self.freq3:.4f}') + " THz")
            
            if self.show3.isChecked():
                
                self.data3.setData(self.spec3)
            else:
                self.data3.clear()
                self.data3.setData()
        else:
            self.data3.clear()
            self.data3.setData()
        
        if self.groupBox4.isChecked():
            
            self.waveData4.setText(str(f'{self.wave4:.4f}') + " nm")
            self.freqData4.setText(str(f'{self.freq4:.4f}') + " THz")

            if self.show4.isChecked():
                
                self.data4.setData(self.spec4)
            else:
                self.data4.clear()
                self.data4.setData()
        else:
            self.data4.clear()
            self.data4.setData()

        if self.groupBox5.isChecked():
            
            self.waveData5.setText(str(f'{self.wave5:.4f}') + " nm")
            self.freqData5.setText(str(f'{self.freq5:.4f}') + " THz")
            
            if self.show5.isChecked():
                
                self.data5.setData(self.spec5)
            else:
                self.data5.clear()
                self.data5.setData()
        else:
            self.data5.clear()
            self.data5.setData()

        if self.groupBox6.isChecked():
            
            self.waveData6.setText(str(f'{self.wave6:.4f}') + " nm")
            self.freqData6.setText(str(f'{self.freq6:.4f}') + " THz")

            if self.show6.isChecked():
                
                self.data6.setData(self.spec6)
            else:
                self.data6.clear()
                self.data6.setData()
        else:
            self.data6.clear()
            self.data6.setData()

        if self.groupBox7.isChecked():
            
            self.waveData7.setText(str(f'{self.wave7:.4f}') + " nm")
            self.freqData7.setText(str(f'{self.freq7:.4f}') + " THz")

            if self.show7.isChecked():
                
                self.data7.setData(self.spec7)
            else:
                self.data7.clear()
                self.data7.setData()
        else:
            self.data7.clear()
            self.data7.setData()

        if self.groupBox8.isChecked():
            
            self.waveData8.setText(str(f'{self.wave8:.4f}') + " nm")
            self.freqData8.setText(str(f'{self.freq8:.4f}') + " THz")

            if self.show8.isChecked():
                
                self.data8.setData(self.spec8)
            else:
                self.data8.clear()
                self.data8.setData()
        else:
            self.data8.clear()
            self.data8.setData()

    def switchModeInfo(self):

        self.data.clear()
        self.data.setData()
        
        while not self.mode:

            time.sleep(0.01)
            if self.currTab == 0:
                continue

            expo = self.wlm.getExposure(self.ch, 1)      # interferometer
            if expo == ErrWlmMissing:
                self.operation()
                return
            if self.expoUp != expo:
                self.expoUp = expo
                self.expoChang = True
            expo = self.wlm.getExposure(self.ch, 2)    # wide interferometer
            if self.expoDown != expo:
                self.expoDown = expo
                self.expoChang = True
            expo = self.wlm.getExposureMode(self.ch)
            if self.expoAut != expo:
                self.expoAut = expo
                self.expoChang = True

            if self.groupBox1.isChecked():

                self.wave1 = self.wlm.getWavelength(1, self.switchTimeData.value()/1000)
                self.freq1 = self.wlm.getFrequency(1)
                
                if self.show1.isChecked():
                    self.spec1 = self.wlm.spectrum(1)
    
            if self.currTab == 0:
                continue

            if self.groupBox2.isChecked():

                self.wave2 = self.wlm.getWavelength(2, self.switchTimeData.value()/1000)
                self.freq2 = self.wlm.getFrequency(2)

                if self.show2.isChecked():
                    self.spec2 = self.wlm.spectrum(2)
    
            if self.currTab == 0:
                continue

            if self.groupBox3.isChecked():
            
                self.wave3 = self.wlm.getWavelength(3, self.switchTimeData.value()/1000)
                self.freq3 = self.wlm.getFrequency(3)

                if self.show3.isChecked():
                    self.spec3 = self.wlm.spectrum(3)
    
            if self.currTab == 0:
                continue

            if self.groupBox4.isChecked():
            
                self.wave4 = self.wlm.getWavelength(4, self.switchTimeData.value()/1000)
                self.freq4 = self.wlm.getFrequency(4)

                if self.show4.isChecked():
                    self.spec4 = self.wlm.spectrum(4)

            if self.currTab == 0:
                continue

            if self.groupBox5.isChecked():
            
                self.wave5 = self.wlm.getWavelength(5, self.switchTimeData.value()/1000)
                self.freq5 = self.wlm.getFrequency(5)

                if self.show5.isChecked():
                    self.spec5 = self.wlm.spectrum(5)
    
            if self.currTab == 0:
                continue

            if self.groupBox6.isChecked():
            
                self.wave6 = self.wlm.getWavelength(6, self.switchTimeData.value()/1000)
                self.freq6 = self.wlm.getFrequency(6)

                if self.show6.isChecked():
                    self.spec6 = self.wlm.spectrum(6)

            if self.currTab == 0:
                continue

            if self.groupBox7.isChecked():
            
                self.wave7 = self.wlm.getWavelength(7, self.switchTimeData.value()/1000)
                self.freq7 = self.wlm.getFrequency(7)

                if self.show7.isChecked():
                    self.spec7 = self.wlm.spectrum(7)

            if self.currTab == 0:
                continue

            if self.groupBox8.isChecked():
            
                self.wave8 = self.wlm.getWavelength(8, self.switchTimeData.value()/1000)
                self.freq8 = self.wlm.getFrequency(8)

                if self.show8.isChecked():
                    self.spec8 = self.wlm.spectrum(8)

            if self.currTab == 0:
                continue

    def analyse(self):

        for i in self.chName:

            if self.chName[i][2] == 1 or self.chName[i][3] == 1:

                self.getSpecificInfo(int(i))
                
                if self.diffCh >= self.refDataInfo[i][1] or self.noPeaks_diff_ch > self.noP_diff_thr:
                    
                    self.ch = int(i)
                    self.locking()

            if not self.relockMode:
                break

        if self.currOverTab == 0:
            self.changeChannel()

        self.analMode = False
        self.relT0 = time.time()
        self.infoMsg.setText(" ")

    def locking(self):

        self.iCurrVal = self.IParam[str(self.ch)][2]
        self.pztCurrVal = self.PztParam[str(self.ch)][2]
        self.corr = True
        self.updWidgetB = True
        while self.updWidgetB:
            time.sleep(0.01)
        self.pztState = False
        self.currState = False
        scan_range = 30
 
        self.lc.setOutput(self.chName[str(self.ch)][6], self.iCurrVal)
        self.lc.setOutput(self.chName[str(self.ch)][5], self.pztCurrVal)
        time.sleep(0.01)
        self.get_info()
        self.corrT0 = time.time()
        self.corrT1 = self.corrT0

        if self.chName[str(self.ch)][2] == 1:
            self.pztState = True

        if self.chName[str(self.ch)][3] == 1:
            self.currState = True
            self.noCorrCurr += 1
            print('No Curr correction ch ', str(self.ch), ': ', self.noCorrCurr, ' at ', time.ctime())

        while ((self.corrT1 - self.corrT0) <= self.stpRelockTimeData.value()) and self.corr and self.currState:

            if self.chName[str(self.ch)][3] == 1 and (self.freq_diff > self.freqStep or self.noPeaks_diff > self.noP_diff_thr):
            
                scan_range = self.current_scan(scan_range)
            else:
                self.currState = False

            self.corrT1 = time.time()

        if ((self.corrT1 - self.corrT0) > self.stpRelockTimeData.value()) and (self.freq_diff > self.freqStep or self.noPeaks_diff > self.noP_diff_thr):

            self.chName[str(self.ch)][3] = 0
            self.currRelock.setChecked(False)

        if self.chName[str(self.ch)][2] == 1 and self.freq_diff >= self.refDataInfo[str(self.ch)][1]:
            
            self.noCorrPzt += 1
            print('No Pzt correction ch ', str(self.ch), ': ', self.noCorrPzt, ' at ', time.ctime())
            self.piezo_change()
        # else:
        #     self.pztState = False

        # self.corrT1 = time.time()

        self.updWidgetA = True
        while self.updWidgetA:
            time.sleep(0.01)
        self.corr = False

    def current_change(self):

        t = 0.01
        thr = self.noP_diff_thr
        step = self.currStep
        min_val = self.IParam[str(self.ch)][0]
        max_val = self.IParam[str(self.ch)][1]
        val = self.IParam[str(self.ch)][2]
        state = self.IParam[str(self.ch)][3]
        prev_val = []
        # prev_freq = self.freq
        itr = 0
        # prev_itr = [0, 0, 0, 0, 0]
        # k = 0

        while self.corr and self.currState:

            if (self.freq_diff > self.freqStep or self.noPeaks_diff > thr) and itr <= 2:

                # k += 1
                # print('first curr: ', k, ' direction change: ', itr)
                if self.freq_diff < self.freqStep:
                    t = 0.1
                    thr = self.noP_diff_thr - self.noP_diff_std
                    step = self.currStep
                else:
                    t = 0.01
                    step = 2*self.currStep

                # if abs(prev_freq - self.freq) > 30*self.freqStep and itr == prev_itr[-5]:

                #     itr += 1
                #     prev_val = []
                #     if state == '+':
                #         state = '-'
                #     else:
                #         state = '+'

                if state == '+':

                    val += step
                    prev_val.append(val)

                    if val < max(prev_val):

                        val = max(prev_val)
                        val += step

                    if val > max_val:

                        val = max_val# - (max_val - min_val)/2
                        state = '-'
                        itr += 1
                        prev_val = []
                else:

                    val -= step
                    prev_val.append(val)

                    if val > min(prev_val):

                        val = min(prev_val)
                        val -= step

                    if val < min_val:

                        val = min_val#max_val - (max_val - min_val)/2
                        state = '+'
                        itr += 1
                        prev_val = []

                self.lc.setOutput(self.chName[str(self.ch)][6], val)
                time.sleep(t)
                # prev_freq = self.freq
                # prev_itr.append(itr)
                self.currSb.setValue(val)
                self.get_info()

            else:
                if self.noPeaks_diff <= thr:
            
                    self.currState = False

                    if state == '+':
                        val += 5*step
                        if val > max_val:
                            val = max_val
                    
                    else:
                        val -= 5*step
                        if val < min_val:
                            val = min_val
                    
                    self.lc.setOutput(self.chName[str(self.ch)][6], val)
                    time.sleep(t)
                    self.get_info()
                    if self.noPeaks_diff <= thr:
                        self.refData[str(self.ch)] = self.spec
                        self.refDataInfo[str(self.ch)][0] = self.noPeaksSpec

                    self.IParam[str(self.ch)][2] = val
                    self.IParam[str(self.ch)][3] = state

                else:
                    val = self.IParam[str(self.ch)][2]
                    self.lc.setOutput(self.chName[str(self.ch)][6], val)
                    time.sleep(t)
                    self.chName[str(self.ch)][3] = 0
                    self.currRelock.setChecked(False)

                self.currSb.setValue(val)

                return

        val = self.IParam[str(self.ch)][2]
        self.lc.setOutput(self.chName[str(self.ch)][6], val)
        time.sleep(t)
        self.currSb.setValue(val)

    def piezo_change(self):

        t = 0.001
        min_val = self.PztParam[str(self.ch)][0]
        max_val = self.PztParam[str(self.ch)][1]
        self.pztCurrVal = self.PztParam[str(self.ch)][2]
        state = self.PztParam[str(self.ch)][3]
        prev_val = []
        prev_freq_diff = self.freq_diff
        itr = 0
        # k = 0

        self.lc.setOutput(self.chName[str(self.ch)][5], self.pztCurrVal)
        time.sleep(t)

        self.get_info()

        while self.corr and self.pztState:

            if self.freq_diff >= (self.refDataInfo[str(self.ch)][1] - self.refDataInfo[str(self.ch)][2]) and \
                self.noPeaks_diff < 1.5*self.noP_diff_thr and itr < 4:

                # k += 1
                # print('piezo: ', k, ' direction change: ', itr)
                if self.freq_diff > prev_freq_diff*1.1:

                    prev_freq_diff = self.freq_diff
                    itr += 1
                    prev_val = []
                    if state == '+':
                        state = '-'
                    else:
                        state = '+'

                if state == '+':

                    self.pztCurrVal += self.PztStep
                    prev_val.append(self.pztCurrVal)

                    if self.pztCurrVal < max(prev_val):

                        self.pztCurrVal = max(prev_val)
                        self.pztCurrVal += self.PztStep

                    if self.pztCurrVal > max_val:

                        self.pztCurrVal = max_val# - (max_val - min_val)/2
                        state = '-'
                        prev_freq_diff = self.freq_diff
                        itr += 1
                        prev_val = []
                else:

                    self.pztCurrVal -= self.PztStep
                    prev_val.append(self.pztCurrVal)

                    if self.pztCurrVal > min(prev_val):

                        self.pztCurrVal = min(prev_val)
                        self.pztCurrVal -= self.PztStep

                    if self.pztCurrVal < min_val:

                        self.pztCurrVal = min_val#max_val - (max_val - min_val)/2
                        state = '+'
                        prev_freq_diff = self.freq_diff
                        itr += 1
                        prev_val = []

                self.lc.setOutput(self.chName[str(self.ch)][5], self.pztCurrVal)
                time.sleep(t)

                self.get_info()

            else:
                if self.freq_diff == -1 or self.noPeaks_diff == -1:

                    self.pztState = False
                elif self.freq_diff < (self.refDataInfo[str(self.ch)][1] - self.refDataInfo[str(self.ch)][2]):

                    self.pztState = False
                    self.PztParam[str(self.ch)][2] = self.pztCurrVal
                    self.PztParam[str(self.ch)][3] = state
                elif itr >= 4:

                    self.chName[str(self.ch)][2] = 0
                    self.pztRelock.setChecked(False)

                self.pztCurrVal = self.PztParam[str(self.ch)][2]
                self.lc.setOutput(self.chName[str(self.ch)][5], self.pztCurrVal)
                time.sleep(t)

                return

        self.pztCurrVal = self.PztParam[str(self.ch)][2]
        self.lc.setOutput(self.chName[str(self.ch)][5], self.pztCurrVal)
        time.sleep(t)

    def current_scan(self, r):

        t = 0.001
        max_itr = self.currStep*r
        min_val = self.IParam[str(self.ch)][0]
        max_val = self.IParam[str(self.ch)][1]
        self.iCurrVal = self.IParam[str(self.ch)][2]

        start_val = -max_itr/2. + self.iCurrVal
        end_val = max_itr/2. + self.iCurrVal

        if start_val < min_val:
            start_val = min_val
        if end_val > max_val:
            end_val = max_val

        # first senario 
        tab1 = np.arange(self.iCurrVal, (end_val + self.currStep), self.currStep)
        tab2 = np.arange(end_val, (start_val - self.currStep), -self.currStep)
        tab3 = np.arange(start_val, (self.iCurrVal + self.currStep), self.currStep)
        tab = np.concatenate((tab1,tab2,tab3),axis=None)

        # second senario
        # tab1 = np.arange(start_val, (end_val + self.currStep), self.currStep)
        # tab2 = np.arange(end_val, (start_val - self.currStep), -self.currStep)
        # tab = np.concatenate((tab1,tab2),axis=None)
        
        for i in tab:
        
            if (self.freq_diff > self.freqStep or self.noPeaks_diff > (self.noP_diff_thr - self.noP_diff_std)) and self.corr:

                # if self.freq_diff < self.freqStep:
                #     t = 0.01
                # else:
                #     t = 0.01

                self.iCurrVal = i
                self.lc.setOutput(self.chName[str(self.ch)][6], self.iCurrVal)
                time.sleep(t)
                self.get_info()

            else:
                if self.freq_diff == -1 or self.noPeaks_diff == -1:

                    self.currState = False
                    self.lc.setOutput(self.chName[str(self.ch)][6], self.IParam[str(self.ch)][2])
                    time.sleep(t)
                elif self.corr:

                    self.currState = False
                    self.refData[str(self.ch)] = self.spec
                    self.refDataInfo[str(self.ch)][0] = self.noPeaksSpec
                    self.IParam[str(self.ch)][2] = self.iCurrVal
                else:
                    self.lc.setOutput(self.chName[str(self.ch)][6], self.IParam[str(self.ch)][2])
                    time.sleep(t)
                return 1
        
        return 2*r

    def piezo_scan(self, r):

        t = 0.001
        max_itr = self.PztStep*r
        min_val = self.PztParam[str(self.ch)][0]
        max_val = self.PztParam[str(self.ch)][1]
        self.pztCurrVal = self.PztParam[str(self.ch)][2]

        start_val = -max_itr/2. + self.pztCurrVal
        end_val = max_itr/2. + self.pztCurrVal

        if start_val < min_val:
            start_val = min_val
        if end_val > max_val:
            end_val = max_val

        # first senario 
        tab1 = np.arange(self.pztCurrVal, (end_val + self.PztStep), self.PztStep)
        tab2 = np.arange(end_val, (start_val - self.PztStep), -self.PztStep)
        tab3 = np.arange(start_val, (self.pztCurrVal + self.PztStep), self.PztStep)
        tab = np.concatenate((tab1,tab2,tab3),axis=None)

        # second senario
        # tab1 = np.arange(start_val, (end_val + self.PztStep), self.PztStep)
        # tab2 = np.arange(end_val, (start_val - self.PztStep), -self.PztStep)
        # tab = np.concatenate((tab1,tab2),axis=None)
        
        for i in tab:
        
            if (self.freq_diff >= (self.refDataInfo[str(self.ch)][1] - self.refDataInfo[str(self.ch)][2]) or self.cavityLock) and self.corr:

                # if self.freq_diff < self.freqStep:
                #     t = 0.01
                # else:
                #     t = 0.01

                self.pztCurrVal = i
                self.lc.setOutput(self.chName[str(self.ch)][5], self.pztCurrVal)
                time.sleep(t)
                self.get_info()

                if self.cavityLock:

                    val = self.lc.getInput(self.chName[str(self.ch)][7])
                    if val > 0.5 and self.freq_diff <= self.refDataInfo[str(self.ch)][1]:
                        
                        self.cavityLock = False
                        self.pztState = False
                        self.PztParam[str(self.ch)][2] = self.pztCurrVal
                        return 1

            else:
                if self.freq_diff == -1 or self.noPeaks_diff == -1:

                    self.pztState = False
                    self.lc.setOutput(self.chName[str(self.ch)][5], self.PztParam[str(self.ch)][2])
                    time.sleep(t)
                elif self.corr:

                    self.pztState = False
                    self.PztParam[str(self.ch)][2] = self.pztCurrVal
                else:
                    self.lc.setOutput(self.chName[str(self.ch)][5], self.PztParam[str(self.ch)][2])
                    time.sleep(t)
                return 1
        
        return 2*r

    def cavityLocking(self, ch):

        val = self.lc.getInput(self.chName[str(ch)][7])
        if val == -11:
            self.sendMsg("assign a correct port for geting information!")
            self.chName[str(ch)][2] == 0
            return

        if val < 0.5 and val > -1 and not self.corr and not self.updMode and not self.analMode and self.mode:

            self.corr = True
            self.ch = ch
            self.iCurrVal = self.IParam[str(self.ch)][2]
            self.pztCurrVal = self.PztParam[str(self.ch)][2]
            self.updWidgetB = True
            while self.updWidgetB:
                time.sleep(0.01)
            self.lc.setOutput(self.chName[str(self.ch)][5], self.pztCurrVal)
            time.sleep(0.01)
            self.get_info()
            scan_range = 30
            self.pztState = True
            self.noCavityCorrPzt += 1
            print('No Cavity correction ch ', str(self.ch), ': ', self.noCavityCorrPzt, ' at ', time.ctime())
            self.corrT0 = time.time()
            self.corrT1 = self.corrT0

            while ((self.corrT1 - self.corrT0) <= self.stpRelockTimeData.value()) and self.corr and self.pztState:

                if self.chName[str(self.ch)][2] == 1:

                    self.cavityLock = True
                    scan_range = self.piezo_scan(scan_range)
                else:
                    self.pztState = False

                self.corrT1 = time.time()

            val = self.lc.getInput(self.chName[str(ch)][7])
            if ((self.corrT1 - self.corrT0) > self.stpRelockTimeData.value()) and (val < 0.5 and val > -1):

                self.chName[str(self.ch)][2] = 0
                self.pztRelock.setChecked(False)

            self.updWidgetA = True
            while self.updWidgetA:
                time.sleep(0.01)
            self.corr = False

    def monitoring(self):

        while not self.monitor and (self.chName['7'][2] == 1 or self.chName['8'][2] == 1):

            if self.chName['7'][2] == 1:

                self.cavityLocking(7)
            
            if self.chName['8'][2] == 1:

                self.cavityLocking(8)

            time.sleep(0.001)

    def notification(self, winIcon, icon, title, text, type):

        msg = QMessageBox()
        msg.setWindowIcon(winIcon)
        msg.setIcon(icon)
        msg.setWindowTitle(title)
        msg.setText(text)
        msg.setStandardButtons(type)

        return msg.exec()

    def eventFilter(self, source, event) -> bool:
        
        if event.type() == QEvent.Wheel and source is self.chData:

            return True
        elif event.type() == QEvent.KeyPress and source is self.chData:

            return True
        elif event.type() == QEvent.Wheel and source is self.upData:

            return True
        elif event.type() == QEvent.Wheel and source is self.downData:

            return True
        elif event.type() == QEvent.Wheel and source is self.currS:

            return True
        elif event.type() == QEvent.Wheel and source is self.currSb:

            return True
        elif event.type() == QEvent.Wheel and source is self.piezoS:

            return True
        elif event.type() == QEvent.Wheel and source is self.piezoSb:

            return True
        elif event.type() == QEvent.Wheel and source is self.updTimeData:

            return True
        elif event.type() == QEvent.Wheel and source is self.relockTimeData:

            return True
        elif event.type() == QEvent.Wheel and source is self.stpRelockTimeData:

            return True
        elif event.type() == QEvent.Wheel and source is self.switchTimeData:

            return True
        else:

            return super().eventFilter(source, event)

    def closeEvent(self, event):

        winIcon = QIcon()
        winIcon.addPixmap(QPixmap(path + "icon\\exit.png"), QIcon.Normal, QIcon.Off)
        icon = QMessageBox.Critical
        title = "Exit"
        text = "Are you sure, you want to Quit?"
        type = QMessageBox.Yes | QMessageBox.Cancel
        returnVal = self.notification(winIcon, icon, title, text, type)

        if returnVal == QMessageBox.Yes:
            
            # self.wlm.exit()
            # if we have threadLock.daemon = True before starting we don't need next line
            # self.corr = False

            self.saveUpd()
            for i in self.refData:
                if self.refDataInfo[i][3] == 1:
                    self.saveFiles(i)

            try:
                
                with open(path + 'setting.txt', 'w') as f:

                    f.write('last value and previous state of Current for laser driver:\n')
                    for i in self.IParam:
                        
                        f.write('channelI ' + i + ':\n' + str(self.IParam[i][2]) + '\n' + self.IParam[i][3] + '\n')
                    
                    f.write('\nlast value and previous state of Piezo for laser driver:\n')
                    for i in self.PztParam:
                        
                        f.write('channelPzt ' + i + ':\n' + str(self.PztParam[i][2]) + '\n' + self.PztParam[i][3] + '\n')
                    
                    f.write('\nlast piezo and current relock mode, update, piezo and current port name for each channel:\n')
                    for i in self.chName:
                        
                        f.write('properties ' + i + ':\n' + str(self.chName[i][2]) + '\n' + str(self.chName[i][3]) + '\n' +\
                                        str(self.chName[i][4]) + '\n' + str(self.chName[i][5]) + '\n' +\
                                        str(self.chName[i][6]) + '\n' + str(self.chName[i][7]) + '\n')
                    
                    f.write('\nupdate (min), relock (s), stop (s), and switch (ms) time values:\n')
                    f.write('timig:\n')
                    f.write(str(self.updTimeData.value()) + '\n')
                    f.write(str(self.relockTimeData.value()) + '\n')
                    f.write(str(self.stpRelockTimeData.value()) + '\n')
                    f.write(str(self.switchTimeData.value()) + '\n')

                    f.close()
            except IOError:
                pass

            event.accept()
        else:
            
            event.ignore()

    def update(self):

        if self.start.text() != "Start":

            try:

                self.updT1 = time.time()
                if ((self.updT1 - self.updT0) >= self.updTimeData.value()*60) and not self.updMode and not self.analMode and not self.corr and self.mode:

                    self.infoMsg.setText("Updating...")
                    self.updMode = True
                    threadUpd = threading.Thread(target=self.updAllInfo)
                    threadUpd.daemon = True
                    threadUpd.start()

                self.relT1 = time.time()
                if ((self.relT1 - self.relT0) >= self.relockTimeData.value()) and self.relockMode and not self.analMode and not self.updMode and not self.corr and self.mode:

                    self.analMode = True
                    threadRel = threading.Thread(target=self.analyse)
                    threadRel.daemon = True
                    threadRel.start()

                if self.relockMode and self.monitor and (self.chName['7'][2] == 1 or self.chName['8'][2] == 1):

                    self.monitor = False
                    threadMonitor = threading.Thread(target=self.monitoring)
                    threadMonitor.daemon = True
                    threadMonitor.start()
                
                if self.corr:

                    self.currSb.setValue(self.iCurrVal)
                    self.piezoSb.setValue(self.pztCurrVal)
                    self.get_info()
                    self.waveData.setText(str(f'{self.wave:.4f}') + " nm")
                    self.freqData.setText(str(f'{self.freq:.4f}') + " THz")
                    self.data.setData(self.spec)

                elif self.currOverTab == 0 and self.currTab == 0 and not self.updMode and not self.analMode:

                    self.get_info()
                    self.waveData.setText(str(f'{self.wave:.4f}') + " nm")
                    self.freqData.setText(str(f'{self.freq:.4f}') + " THz")
                    self.data.setData(self.spec)

                elif self.currOverTab == 0 and self.currTab == 1 and self.mode and not self.updMode and not self.analMode:

                    self.mode = False
                    threadMode = threading.Thread(target=self.switchModeInfo)
                    threadMode.daemon = True
                    threadMode.start()

                elif self.currOverTab == 0 and self.currTab == 1 and not self.mode and not self.updMode and not self.analMode:

                    self.updSwitchMode()
                    
                if self.updWidgetB:
                    
                    self.infoMsg.setText("Relocking...")
                    self.chData.setCurrentIndex(self.ch - 1)
                    self.chData.setDisabled(True)
                    self.tabs.setCurrentIndex(0)
                    self.tabs.setTabEnabled(1, False)
                    self.initialTabs.setCurrentIndex(0)
                    self.changeChannel()
                    self.piezoS.setDisabled(True)
                    self.piezoSb.setDisabled(True)
                    self.currS.setDisabled(True)
                    self.currSb.setDisabled(True)
                    self.updWidgetB = False

                if self.updWidgetA:

                    self.infoMsg.setText(" ")
                    self.chData.setDisabled(False)
                    self.tabs.setTabEnabled(1, True)
                    self.piezoS.setDisabled(False)
                    self.piezoSb.setDisabled(False)
                    self.currS.setDisabled(False)
                    self.currSb.setDisabled(False)
                    self.updWidgetA = False

                if self.currOverTab == 0 and self.expoChang:
                    
                    self.expoChang = False
                    self.upData.setValue(self.expoUp)
                    self.downData.setValue(self.expoDown)
                    self.automaticExpo.setChecked(self.expoAut)
            except AttributeError:
                pass



if __name__ == '__main__':

    app = QApplication(sys.argv)
    ui = Ui_MainWindow()
    ui.show()
    ui.getInitialValues()
    timer = QTimer()
    timer.setInterval(10)
    timer.timeout.connect(ui.update)
    timer.start()
    sys.exit(app.exec())


