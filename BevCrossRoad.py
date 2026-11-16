# Created by: ww 2024/5/13, 基于选取的区域及对应点，生成跨场景BEV像素—物理图
import numpy as np
import cv2 as cv
from PyQt5.QtCore import pyqtSignal, QObject
import MathTool as mt


class BevCrossRoad(QObject):
    """ 基于手选两场景区域及对应点，生成跨场景BEV像素—物理图 """
    send_win_flush = pyqtSignal(object)  # 用于刷新主界面信号

    def __init__(self):
        super(BevCrossRoad, self).__init__()
        self.bev_xy_range = [0.0, 0.0, 0.0, 0.0]  # 记录bev视图坐标xy的范围，x_min,x_max,y_min,y_max，单位为mm
        self.bev_cross_img = None  # 生成的bev视角跨场景图像
        self.R_t = [[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], [0.0, 0.0, 0.0]]  # 初始化R，t，恒等变换

    def bev_img_gen(self, p_area, P_area, img):
        """ 单场景bev视角图像生成 """
        h_gen = 150  # 生成的bev视角图像高度pix(固定值，超参数)
        rat_lh = abs((P_area[0][1]-P_area[1][1])/(P_area[0][0]-P_area[2][0]))  # 区域的物理长宽比
        w_gen = round(h_gen * rat_lh)  # 生成的bev视角图像宽度
        dst = np.array([[0, 0], [w_gen-1, 0], [w_gen-1, h_gen-1], [0, h_gen-1]], dtype=np.float32)  # 左上，右上，右下，左下
        p_area = np.array(p_area, dtype=np.float32)
        matrix = cv.getPerspectiveTransform(p_area, dst)  # 计算透视变换矩阵，四个点通常按照从左上角开始，顺时针的顺序指定
        bev_img = cv.warpPerspective(img, matrix, (w_gen, h_gen))  # 应用透视变换
        return bev_img

    def coordinate_system_one(self, P_correspond1, P_correspond2):
        """ 两个坐标系统一，一般左侧视图为来向视图(y负)，右侧视图为去向视图(y正)
         P_correspond1:n×3的list, P_correspond2:n×3的list，两组点相差整数倍15米(一组虚实线15米)"""
        P1_y, P2_y = P_correspond1[:, 1], P_correspond2[:, 1]  # 取出两组对应点的所有y坐标
        diff_y = P2_y - P1_y  # 求对应点之间的物理间距
        # diff_y = np.round(diff_y / 15000) * 15000  # 将间距调整为15米的整数倍

        # 获得P_correspond1在scene2下的表示
        P_correspond1_in2 = P_correspond2.copy()
        P_correspond1_in2[:, 1] = P2_y - diff_y
        # 获得左视图—右视图的三维刚体变换矩阵
        self.R_t[0], self.R_t[1] = mt.compute_transformation(P_correspond1, P_correspond1_in2)

    def bev_img_cross_gen(self, imgs, set_mark_dicts):
        """ 两场景图像及标记点生成跨场景bev视角图像，及统一坐标系  """
        p_areas = [set_mark_dicts[0]['p_area'], set_mark_dicts[1]['p_area']]
        P_areas = [np.array(set_mark_dicts[0]['P_area']), np.array(set_mark_dicts[1]['P_area'])]
        P_corresponds = [np.array(set_mark_dicts[0]['P_correspond']), np.array(set_mark_dicts[1]['P_correspond'])]
        self.bev_xy_range[2] = min([min(P_areas[0][:, 1]), min(P_areas[1][:, 1])])  # 两场景最小y值
        self.bev_xy_range[3] = max([max(P_areas[0][:, 1]), max(P_areas[1][:, 1])])  # 两场景最大y值
        self.bev_xy_range[:2] = min(P_areas[1][:, 0]), max(P_areas[1][:, 0])  # 以第二个场景为基准，获得x值范围
        # 先对两场景三维坐标系做统一化变换
        self.coordinate_system_one(*P_corresponds)
        # 再生成跨场景bev视图
        bev_imgs = []  # 记录两个bev视角图
        for i in range(2):
            p_area, P_area = p_areas[i], P_areas[i]
            bev_img = self.bev_img_gen(p_area, P_area, imgs[i])  # 生成选择区域的bev视角图
            ratio = abs(min(abs(P_area[0][1]), abs(P_area[1][1])) / (P_area[0][1] - P_area[1][1]))  # 盲区与选择区域的物理长度比例
            blind_spot_img = np.zeros((bev_img.shape[0], round(bev_img.shape[1]*ratio), 3), dtype=np.uint8)  # 新建比例尺的盲区图像
            if max(P_area[0][1], P_area[1][1]) < 0:  # 判断相机拍摄道路角度，对着车流方向，y值为负，顺着车流方向，y值为正
                bev_imgs.append(np.hstack((bev_img, blind_spot_img)))
            else:
                bev_imgs.append(np.hstack((blind_spot_img, bev_img)))
        self.bev_cross_img = np.hstack((bev_imgs[0], bev_imgs[1]))  # 拼接两个bev视角图
        y_min, y_max = round(self.bev_xy_range[2] / 1000), round(self.bev_xy_range[3] / 1000)  # 取整
        w = round((self.bev_xy_range[1] - self.bev_xy_range[0]) / 1000)  # 路宽度
        self.bev_cross_img = mt.draw_img_axes(self.bev_cross_img, y_min, y_max, w)  # 绘制物理坐标轴

    def pot_in_bevcross_img(self, view_id, Pot):
        """ 对于在视图中选取的三维点，在bev中绘制以验证，注意：view_id为1时，直接等比例绘制，view_id为0时，需要变换坐标系后再绘制  """
        x_range = abs(self.bev_xy_range[1] - self.bev_xy_range[0])
        y_range = abs(self.bev_xy_range[3] - self.bev_xy_range[2])
        u, v = 0, 0  # 初始化Pot对应的Bev视图像素点
        if view_id == 0:  # 场景1，先变换坐标系
            R, t = self.R_t  # 取出旋转矩阵和平移向量
            Pot = R @ Pot + t  # 变换坐标系
        v = round((1 - (Pot[0] - self.bev_xy_range[0]) / x_range) * self.bev_cross_img.shape[0])  # 1-...，是因为v与x轴反向
        u = round((Pot[1] - self.bev_xy_range[2]) / y_range * self.bev_cross_img.shape[1])
        # 在Bev图上打印对应点输出
        cv.circle(self.bev_cross_img, (u, v), 3, (0, 0, 255), -1)