# 提取两场景BEV交互信息，Created by: ww 2024/4/20
import json
import os
import win32api  # 提示框，win32api.MessageBox(0, "xxx", "提示", win32con.MB_ICONWARNING)
import win32con
import numpy as np
import cv2 as cv
from PyQt5.QtGui import QTransform, QImage, QPixmap, QColor, QPen, QFont, QPainter
from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSignal, QObject, QRect, Qt, QPoint
from Dialog_Subs.Redund_UI import Ui_Dialog_Redund as UI_Redund
import MathTool as mt


class ManuMarkDetect(QGraphicsView, QWidget, QObject):
    """ 手选图像上标识类，继承QGraphicsView基类可在QT界面中与图像做交互
    (为了方便做选点交互，QGraphicsView在此类中建立，没有在MainWin文件中建立) """
    send_win_flush = pyqtSignal(object)  # 用于刷新主界面信号

    def __init__(self, parent):
        """ parent为QGraphicsView的父窗口 """
        super(ManuMarkDetect, self).__init__(parent)
        # 设置QGraphicsView
        self.setGeometry(QRect(0, 20, 921, 561))
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setMouseTracking(True)
        self.show_scene = QGraphicsScene(self)
        self.setBackgroundBrush(QColor(0, 0, 0))  # 图像显示初始化背景为黑色
        self.img_view(np.zeros((200, 200, 1), dtype=np.uint8))

        # 类变量
        self.ratio = np.array([1.0, 1.0])
        self.flag_interact = 0  # 0-不交互/1-区域设定操作/2-场景对应点选取操作/3-...
        self.p_pot, self.P_pot = [], []  # 记录手选像素坐标和反算的世界坐标
        self.img = None
        self.file_dir = None  # 当前场景文件夹路径
        self.principle_point = None  # 图像主点
        self.calib_params = None  # 标定参数
        self.view_id = 0  # 当前视图的id

    def img_view(self, img, flag_adapt=True):
        """ 设置图像在view中自适应显示 """
        def adaptive_display():
            """ 设置自适应填充view """
            width, height = self.size().width(), self.size().height()
            scale_x, scale_y = width / cols, height / rows  # 计算水平和垂直缩放比例
            self.setTransform(QTransform().scale(scale_x, scale_y))  # 设置缩放
            self.ratio = np.array([scale_x, scale_y])

        img_copy = img.copy()
        rows, cols, depth = img_copy.shape
        nimg = QImage(img_copy.data, cols, rows, cols * depth, QImage.Format_BGR888)  # numpy矩阵转换成QImage
        pix = QPixmap.fromImage(nimg)
        pix_item = QGraphicsPixmapItem(pix)
        self.show_scene = QGraphicsScene(self)  # 每次都要创建新的Scene，否则setTransform后比例不对
        self.show_scene.addItem(pix_item)
        self.setScene(self.show_scene)
        if flag_adapt:  # 若需要自适应在视图中显示
            adaptive_display()

    def wheelEvent(self, event):
        """重写滚轮事件"""
        if not self.flag_interact:
            return
        wheelvalue = event.angleDelta().y()
        r = wheelvalue / 1200.0 + 1
        self.ratio *= r  # 记录累积缩放倍数
        self.scale(r, r)

    def msg_print(self, msg, font, color, pot_x, pt_y):
        """ 在scene的给定位置(pot_x, pt_y)输出信息msg """
        text_item = QGraphicsTextItem()
        text_item.setFont(font)  # 设置字体
        text_item.setDefaultTextColor(color)  # 设置字体颜色
        text_item.setPlainText(msg)  # 设置文本
        text_item.setPos(pot_x, pt_y)  # 将文本项放置在设定位置
        self.show_scene.addItem(text_item)

    def paint_p_area(self):
        """ 在scene中绘制选择区域p_area, repaint_img函数的子函数"""
        p_area = self.load_jsonSetMarkDict()['p_area']  # 加载设置标识字典
        p_area_img = self.img.copy()
        for i in range(4):
            cv.line(p_area_img, p_area[i], p_area[(i + 1) % 4], color=(0, 255, 0), thickness=2)
        return p_area_img

    def repaint_img(self, msgs, pos):
        """ 重绘图像，mousePressEvent的子函数 """
        self.scene().clear()  # 每次鼠标事件都要对scene重新绘制
        if self.flag_interact == 1:
            mask_img = mt.area_mask(self.p_pot, self.img)  # 生成选择区域的掩膜标识
            self.img_view(mask_img)  # 重绘img
        elif self.flag_interact == 2:
            self.img_view(self.img, flag_adapt=False)  # 重绘img, 同时比例保持不变
        elif self.flag_interact == 3:
            self.img_view(self.paint_p_area(), flag_adapt=False)  # 重绘带p_area的img，同时比例保持不变
        pen = QPen()
        pen.setColor(QColor(255, 0, 0))
        for pot in pos:  # 画点
            pen.setWidth(4)
            self.scene().addEllipse(pot[0] - 4, pot[1] - 4, 8, 8, pen)
        for i in range(len(msgs)):  # 在对应位置打印消息
            if self.flag_interact == 2:  # 对应点设置时，需要打印点顺序
                self.msg_print(str(i), QFont('Arial', 25), QColor(255, 0, 0), pos[i][0], pos[i][1])
            self.msg_print(msgs[i], QFont('Arial', 15), QColor(0, 0, 0), pos[i][0]+20, pos[i][1])

    def multi_lineEdit(self):
        """ 重写子UI，实现多行文本框，mousePressEvent的子函数 """
        alp, awp = 100, 15  # 默认区域长度、宽度(m)
        q_dialog, dialog = QDialog(), UI_Redund()
        dialog.setupUi(q_dialog)  # 继承QDialog()，使得dialog具有show()方法
        q_dialog.show()
        if q_dialog.exec() == QDialog.Accepted:  # 提取用户交互的参数
            alp = float(dialog.lineEdit_Lenth.text())  # 记录区域长度(area_lengh_physic)
            awp = float(dialog.lineEdit_Width.text())  # 记录区域宽度(area_width_physic)
        return alp, awp

    def mousePressEvent(self, event):
        """ 重写鼠标事件，基于选择的不同的标识类型做不同的操作 """
        if not self.flag_interact or event.button() not in (Qt.LeftButton, Qt.RightButton):  # 不需要交互时返回
            return
        # 鼠标左键进行选点
        if event.button() == Qt.LeftButton:
            p = self.mapToScene(event.pos())  # 鼠标选点映射回原图像
            if self.flag_interact == 1:  # 区域设置，只需记录一个起始点，用户输入区域长度和宽度后，自动计算出其他三个点
                alp, awp = self.multi_lineEdit()  # 获取用户交互的区域长，宽参数
                self.p_pot, self.P_pot = mt.area_detect_generate(self.calib_params, [int(p.x()), int(p.y())],  # 区域4点坐标
                                                                 alp * 1000, awp * 1000, self.principle_point)
                for p in self.p_pot:
                    polygon = [(0, 0), (self.img.shape[1], 0), (self.img.shape[1], self.img.shape[0]), (0, self.img.shape[0])]
                    if not mt.is_point_in_polygon(p, polygon):
                        win32api.MessageBox(0, "选择区域超出图像边界，请重新选择！", "提示", win32con.MB_ICONWARNING)
                        return
            if self.flag_interact in [2, 3]:  # 场景对应点设置/交互验证
                pot, polygon = [int(p.x()), int(p.y())], self.load_jsonSetMarkDict()["p_area"]
                if not mt.is_point_in_polygon(pot, polygon):
                    win32api.MessageBox(0, "选择点超出设置区域，请重新选择！", "提示", win32con.MB_ICONWARNING)
                    return
                self.p_pot.append(pot)
                phy_p = mt.uv_to_xyz(self.calib_params, self.p_pot[-1], 0, self.principle_point, True)
                self.P_pot.append([int(item) for item in phy_p])  # 记录对应点的世界坐标，并转换成整数
                if self.flag_interact == 3:  # 传回主界面当前
                    self.send_win_flush.emit(["cross_scenes", self.view_id, self.P_pot[-1]])  # 信号传回主界面

        # 鼠标右键清空，只对flag_interact==2有效，清空最近一次选点
        elif event.button() == Qt.RightButton:
            if not len(self.p_pot) or self.flag_interact == 1:  # 若点全删除完了或区域设定操作
                return
            if self.flag_interact == 2:  # 删除最近一次选点
                del self.p_pot[-1]
                del self.P_pot[-1]
        else:
            return

        # 点击左/右键需要重新显示，绘制全部点及线段
        msg = [str(item[:2]) for item in self.P_pot]  # 在图像中显示点对应的物理值，只显示(x,y)值
        self.repaint_img(msg, self.p_pot)

    def keyPressEvent(self, event):
        """ 重写键盘事件，点击Enter键保存手选标识结果 """
        if not self.flag_interact:
            return
        if event.text() == '\r' or '\n':  # 点击Enter键
            if self.flag_interact == 1:  # 区域设置保存
                self.p_pot, self.P_pot = mt.area_pots_sort(self.p_pot, self.P_pot)  # 重新排序
                self.update_jsonSetMarkDict("p_area", self.p_pot)
                self.update_jsonSetMarkDict("P_area", self.P_pot)
            if self.flag_interact == 2:  # 对应点设置保存
                self.update_jsonSetMarkDict("p_correspond", self.p_pot)
                self.update_jsonSetMarkDict("P_correspond", self.P_pot)
            self.send_win_flush.emit(["select_operate", 0])  # 信号传回主界面，标识下拉框选0
            self.flag_interact = 0  # 设置为不交互
            self.load_jsonSetMark_picshow(self.img)  # 显示所有标识

    def load_jsonCalib(self, calib_path):
        """ 加载CalibResult.json文件，返回字典calib_matrix """
        with open(calib_path, 'r') as f:
            calib_params_dict = json.load(f)  # 加载标定参数
            img_shape = self.img.shape[:2]
            self.principle_point = [img_shape[1] // 2, img_shape[0] // 2]  # 图像主点
            self.calib_params = [calib_params_dict[key] for key in ["focal", "fi", "theta", "cam_h"]]  # 从字典中提取参数值
            # 显示标定参数字符
            calib_text = 'f: ' + str(round(self.calib_params[0], 2)) + ', fi: ' + str(round(self.calib_params[1], 2)) + \
                         ', theta: ' + str(round(self.calib_params[2], 2)) + ', h: ' + str(round(self.calib_params[3], 2))
            self.img = cv.putText(self.img, calib_text, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)

    def creat_jsonSetMarkDict(self):
        """ 创建空白场景设置字典文件(转BEV视图用) """
        # p_area:设定区域的4个顶点像素坐标；P_area:设定区域的4个顶点世界坐标；
        # p_correspond:坐标变换对应点像素点集；P_correspond:坐标变换对应点物理点集；
        data = {"p_area": [], "P_area": [], "p_correspond": [], "P_correspond": []}
        with open(self.file_dir + "/SetMarkDict.json", "w") as f:
            json.dump(data, f)

    def update_jsonSetMarkDict(self, dict_key, dict_value):
        """ 根据用户传入的字典键/值对SetMarkDict.json文件更新 """
        with open(self.file_dir + "/SetMarkDict.json", "r") as f:
            data = json.load(f)
        data[dict_key] = dict_value
        with open(self.file_dir + "/SetMarkDict.json", "w") as f:
            json.dump(data, f)

    def load_jsonSetMarkDict(self):
        """ 对SetMarkDict.json文件，返回字典set_mark_dicts """
        with open(self.file_dir + "/SetMarkDict.json", "r") as f:
            set_mark_dict = json.load(f)
        return set_mark_dict

    def load_jsonSetMark_picshow(self, img):
        """ 加载SetMarkDict.json文件，并将设置的标识显示在图像上 """
        set_dict = {"p_area": [], "P_area": [], "p_correspond": [], "P_correspond": []}
        with open(self.file_dir + "/SetMarkDict.json", "r") as f:
            set_dict = json.load(f)
        p_area, P_area = set_dict["p_area"], set_dict["P_area"]
        p_cod, P_cod = set_dict["p_correspond"], set_dict["P_correspond"]
        self.scene().clear()  # 对scene重新绘制
        mask_img = mt.area_mask(p_area, img) if len(p_area) else img  # 生成选择区域的掩膜标识
        self.img_view(mask_img)  # 显示图像
        pen = QPen()
        pen.setColor(QColor(255, 0, 0))
        for pot, Pot in zip(p_area + p_cod, P_area + P_cod):  # 画点
            pen.setWidth(4)
            self.scene().addEllipse(pot[0] - 4, pot[1] - 4, 8, 8, pen)
            self.msg_print(str(Pot), QFont('Arial', 15), QColor(0, 0, 0), pot[0] + 20, pot[1])  # 在对应位置打印坐标
        for i in range(len(p_cod)):  # 在对应位置打印对应点排序
            self.msg_print(str(i), QFont('Arial', 25), QColor(255, 0, 0), p_cod[i][0], p_cod[i][1])
