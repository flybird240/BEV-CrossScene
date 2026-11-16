# Created by: ww 2020/8/11，界面与逻辑分离，主窗口逻辑代码
import os
import sys
import cv2 as cv
import numpy as np

import win32api  # 提示框，win32api.MessageBox(0, "xxx", "提示", win32con.MB_ICONWARNING)
import win32con

from MainDetect_UI import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon
from PyQt5.QtGui import QPixmap, QImage

from ManualMarkDetect import ManuMarkDetect
from BevCrossRoad import BevCrossRoad


class Main(QMainWindow, Ui_MainWindow):
    """ 重写主窗体类 """
    def __init__(self, parent=None):
        super(Main, self).__init__(parent)
        self.setupUi(self)  # 初始化窗体显示
        self.center()  # 窗口居中显示
        self.setWindowIcon(QIcon("crossBEV.png"))  # 设置窗口图标
        self.label_CrossScenes.setScaledContents(True)  # 设置在label中自适应显示图片
        self.label_CrossScenes.setStyleSheet("QLabel{background-color:rgb(0,0,0);}")  # 初始黑化图像显示区域
        self.mamDet = [ManuMarkDetect(self.groupBox_PicShow1), ManuMarkDetect(self.groupBox_PicShow2)]  # 手选标识对象
        self.bevCross = BevCrossRoad()  # 新建立一个跨场景bev对象
        self.wid_dict = {"select_operate": self.comboBox_selectOperate, "cross_scenes": self.label_CrossScenes}  # 界面刷新所需字典

    def center(self):
        """定义一个函数使得窗口居中显示"""
        screen = QDesktopWidget().screenGeometry()  # 获取屏幕坐标系
        size = self.geometry()
        new_left = (screen.width() - size.width()) / 2
        new_top = (screen.height() - size.height()) / 2
        self.move(int(new_left), int(new_top))

    def closeEvent(self, event):
        """ 重写closeEvent方法，event: close()触发的事件  """
        reply = QtWidgets.QMessageBox.question(self, "info", "是否确定退出程序？",
                                               QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                               QtWidgets.QMessageBox.Yes)
        if reply == QtWidgets.QMessageBox.Yes:
            event.accept()
            sys.exit(0)
        else:
            event.ignore()

    def oncombox_select_scene(self, index):
        """ 选择相机标定场景图象 """
        if index == 0:
            return
        # 若为选择场景操作
        file_path, _ = QFileDialog.getOpenFileName(self, "选择图像", os.getcwd() + '/traffic_scenes_dataset',
                                                   "Images (*.bmp *.jpg *.png);;All (*)")
        calib_path = os.path.dirname(file_path) + "/CalibResult_OP_pso.json"  # 场景标定结果
        if not file_path or not os.path.exists(calib_path):  # 没有图像或标定文件
            win32api.MessageBox(0, "图像加载失败或不存在标定文件！", "提示", win32con.MB_ICONWARNING)
            self.comboBox_selectScene.setCurrentIndex(0)
            return

        i = index - 1  # 简化变量名
        self.mamDet[i].view_id = i  # 当前视图id（0或1）
        self.mamDet[i].img = cv.imdecode(np.fromfile(file_path, np.uint8), cv.IMREAD_COLOR)  # 支持中文路径
        self.mamDet[i].load_jsonCalib(calib_path)  # 加载标定参数
        self.mamDet[i].send_win_flush.connect(self.slot_win_flush)  # 建立信号与槽函数连接，实时刷新界面
        self.mamDet[i].file_dir = os.path.dirname(file_path)  # 当前文件夹路径
        if not os.path.exists(os.path.dirname(file_path) + "/SetMarkDict.json"):  # 场景设置字典文件
            self.mamDet[i].creat_jsonSetMarkDict()

        # 设置两窗口，当前窗口加载带设置标识的单场景，另一窗口设置为不可编辑
        self.mamDet[i].load_jsonSetMark_picshow(self.mamDet[i].img)  # 加载显示设置标识的图像
        self.views_set([i, (i + 1) % 2], [Qt.CrossCursor, Qt.ArrowCursor], [0, 0])

    def views_set(self, view_id_list, cursor_list, flag_interact_list):
        """ 设置多个场景图象交互操作方式 """
        for i in range(len(view_id_list)):
            self.mamDet[view_id_list[i]].setCursor(cursor_list[i])  # 设置鼠标光标
            self.mamDet[view_id_list[i]].flag_interact = flag_interact_list[i]  # 设置交互操作方式

    def oncombox_select_operate(self, index):
        """ 选择场景图象重建操作方式 """
        if index == 3:
            self.views_set([0, 1], [Qt.CrossCursor, Qt.CrossCursor], [3, 3])  # 交互验证时，设置两view都可交互
            self.comboBox_selectScene.setCurrentIndex(0)  # 设置“选择场景”下拉框默认选择第一个场景
            for i in range(2):  # 两view重显示带p_area区域的scene
                self.mamDet[i].p_pot, self.mamDet[i].P_pot = [], []  # 换交互操作时，清空之前的点集
                self.mamDet[i].img_view(self.mamDet[i].paint_p_area())
                self.mamDet[i].flag_interact = index  # 交互操作类型
            return
        scene_id = self.comboBox_selectScene.currentIndex() - 1  # 场景序号
        self.mamDet[scene_id].flag_interact = index  # 交互操作类型
        self.mamDet[scene_id].p_pot, self.mamDet[scene_id].P_pot = [], []  # 换交互操作时，清空之前的点集
        self.mamDet[scene_id].img_view(self.mamDet[scene_id].img)  # 换交互操作时，重新显示场景图象

    def onbuttonclick_cross_bev(self):
        """ 跨场景BEV视图重建按钮 """
        set_mark_dicts = []
        for i in range(2):
            set_mark_dict = self.mamDet[i].load_jsonSetMarkDict()  # 加载设置标识字典
            if not len(set_mark_dict["p_area"]) or not len(set_mark_dict["P_correspond"]):  # 未完成设置标识
                win32api.MessageBox(0, "请先设置两场景标识！", "提示", win32con.MB_ICONWARNING)
                return
            set_mark_dicts.append(set_mark_dict)

        self.bevCross.bev_img_cross_gen([self.mamDet[0].img, self.mamDet[1].img], set_mark_dicts)  # 生成跨场景bev视图
        self.bev_cross_scene_show(self.bevCross.bev_cross_img)  # 在label中显示跨场景BEV视图

    def bev_cross_scene_show(self, img):
        """ 在label中显示跨场景BEV视图 """
        if img.shape[-1] == 3:
            qimage = QImage(img.data.tobytes(), img.shape[1], img.shape[0], img.shape[1]*3, QImage.Format_RGB888).rgbSwapped()
        else:
            qimage = QImage(img.data.tobytes(), img.shape[1], img.shape[0], img.shape[1], QImage.Format_Indexed8)
        self.label_CrossScenes.setPixmap(QPixmap.fromImage(qimage))

    def slot_win_flush(self, msg):
        """ 刷新主界面显示信息 """
        if isinstance(self.wid_dict[msg[0]], QComboBox):
            self.wid_dict[msg[0]].setCurrentIndex(int(msg[1]))  # QComboBox设置值
        if isinstance(self.wid_dict[msg[0]], QLabel):
            view_id, Pot = msg[1:]  # 场景id和在该场景下选择的三维点
            self.bevCross.pot_in_bevcross_img(view_id, Pot)
            self.bev_cross_scene_show(self.bevCross.bev_cross_img)  # QLabel，更新在BEV图中的显示


if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = Main()
    win.show()
    print(app.exec_())
    sys.exit(app.exec_())
