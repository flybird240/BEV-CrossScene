# Revised by: @Juliet tang 2021/8/25
# 包含所有用到的工具函数，仅保留与本项目使用到的函数

import numpy as np
import math
from PIL import Image, ImageDraw, ImageFont
import json
import cv2 as cv
from scipy.linalg import svd


class NpEncoder(json.JSONEncoder):
    """继承json.JSONEncoder，自定义序列化方法(可打包numpy格式数据)"""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(NpEncoder, self).default(obj)


def calib_param_to_matrix_basic(calib_params, principal_point, flag_road):
    """ 将标定参数转换为变换矩阵，参考论文公式(5),三维旋转矩阵参考：https://blog.csdn.net/ycrsw/article/details/123624845
    focal:焦距，fi:俯仰角(rad)，theta:旋转角(rad)，h:相机高度(mm)，pcx:主点u，pcy:主点v
    return: world -> image 变换矩阵 """
    focal, fi, theta, h = calib_params
    theta = theta if theta < 0 else theta + math.pi  # 绕z轴逆时针旋转角度，如果大于0，则y轴旋转至与道路方向相反(车流方向)(ww24-5-17)
    pc_u, pc_v = principal_point
    K = np.array([focal, 0, pc_u, 0, focal, pc_v, 0, 0, 1]).reshape(3, 3).astype(np.float32)
    Rx = np.array([1, 0, 0, 0, -math.sin(fi), -math.cos(fi), 0, math.cos(fi), -math.sin(fi)]).reshape(3, 3).astype(np.float32)
    Rz = np.array([math.cos(theta), -math.sin(theta), 0, math.sin(theta), math.cos(theta), 0, 0, 0, 1]).reshape(3,3).astype(np.float32)
    R = np.dot(Rx, Rz) if flag_road else Rx  # y轴是否沿道路方向
    T = np.array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -h]).reshape(3, 4).astype(np.float32)
    trans = np.dot(R, T)
    H = np.dot(K, trans)
    return H, R, T, K


def uv_xyz_trans(calib_matrix, u, v, z):
    """ 图像坐标--->世界坐标，本函数为矩阵的逆运算,世界坐标z需要指定，坐标单位：mm，calib_matrix：3×4矩阵 """
    h11, h12, h13, h14 = calib_matrix[0]
    h21, h22, h23, h24 = calib_matrix[1]
    h31, h32, h33, h34 = calib_matrix[2]

    a11, a12 = h11 - u * h31, h12 - u * h32
    a21, a22 = h21 - v * h31, h22 - v * h32
    b1 = u * (h33 * z + h34) - (h13 * z + h14)  # 与之前版本有修改
    b2 = v * (h33 * z + h34) - (h23 * z + h24)
    x = (b1 * a22 - a12 * b2) / (a11 * a22 - a12 * a21)
    y = (a11 * b2 - b1 * a21) / (a11 * a22 - a12 * a21)
    return -x, y, z  # -x变为左手坐标系，方便表示(沿道路宽度为正方向)(ww24-5-17)


def xyz_uv_trans(calib_matrix, x, y, z):
    """ 世界坐标--->图像坐标，利用标定矩阵直接算, 世界坐标单位：mm """
    x = -x  # -x变为左手坐标系，方便表示(沿道路宽度为正方向)(ww24-5-17)
    h11, h12, h13, h14 = calib_matrix[0]
    h21, h22, h23, h24 = calib_matrix[1]
    h31, h32, h33, h34 = calib_matrix[2]

    u = (h11 * x + h12 * y + h13 * z + h14) / (h31 * x + h32 * y + h33 * z + h34)
    v = (h21 * x + h22 * y + h23 * z + h24) / (h31 * x + h32 * y + h33 * z + h34)
    return u, v


def uv_to_xyz(calib_params, pix_point, z, principal_point, flag_road=False):
    """ 图像坐标--->世界坐标集成函数, 世界坐标z需要指定，坐标单位：mm
     calib_params: 标定参数组[f, fi, theta, h];  pix_point：原图像像素[u, v];
     principal_point:图像主点(中心点)坐标[pc_u, pc_v];  flag_road：False-原论文世界坐标系，True-y轴沿道路方向"""
    H, _, _, _ = calib_param_to_matrix_basic(calib_params, principal_point, flag_road)
    u, v = pix_point
    phy_point = uv_xyz_trans(H, u, v, z)
    return phy_point


def xyz_to_uv(calib_params, phy_point, principal_point, flag_road=False):
    """ 世界坐标--->图像坐标集成函数, 坐标单位：mm
     calib_params: 标定参数组[f, fi, theta, h];  phy_point：空间坐标点[x, y, z];
     principal_point:图像主点(中心点)坐标[pc_u, pc_v];  flag_road：False-原论文世界坐标系，True-y轴沿道路方向 """
    H, _, _, _ = calib_param_to_matrix_basic(calib_params, principal_point, flag_road)  # 基于标定参数计算标定矩阵
    x, y, z = phy_point
    pix_point = xyz_uv_trans(H, x, y, z)
    # pix_point += np.array(principal_point)  # 移回原点在左上角的图像坐标系
    return pix_point


def area_detect_generate(calib_params, start_point, alp, awp, principal_point, flag_road=True):
    """ 根据用户选择的区域，返回该区域的像素顶点和世界坐标顶点
    calib_params: 标定参数组[f, fi, theta, h];  start_point：图像坐标点[u, v]，区域起始点;  alp, awp: 区域物理长度与宽度(mm)；
    principal_point:图像主点(中心点)坐标[pc_u, pc_v];  flag_road：False-原论文世界坐标系，True-y轴沿道路方向 """
    def tuple_to_integers(input_tuple):
        # 使用生成器表达式将元组中的每个元素转换为整数
        return tuple(int(item) for item in input_tuple)
    P0 = np.array(uv_to_xyz(calib_params, start_point, 0, principal_point, flag_road))  # 计算起始点的世界坐标
    P1, P2, P3 = P0 + np.array((0, alp, 0)), P0 + np.array((awp, alp, 0)), P0 + np.array((awp, 0, 0))  # 其他点的世界坐标
    p1 = xyz_to_uv(calib_params, P1.tolist(), principal_point, flag_road)
    p2 = xyz_to_uv(calib_params, P2.tolist(), principal_point, flag_road)
    p3 = xyz_to_uv(calib_params, P3.tolist(), principal_point, flag_road)
    p_list = [start_point, p1, p2, p3, P0, P1, P2, P3]
    for i in range(len(p_list)):  # 对所有数据整数化
        p_list[i] = tuple_to_integers(p_list[i])
    return p_list[0:4], p_list[4:]


def area_mask(p_area, img):
    """ 对图像img中的不规则区域p_area，生成该区域的掩膜，以区分其他区域 """
    img = cv.cvtColor(img, cv.COLOR_BGR2BGRA)  # 确保图像是BGRA格式，这样才有Alpha通道
    mask = np.zeros_like(img)  # 创建与原图相同大小的掩膜，初始完全透明
    points = np.array(p_area, dtype=np.int32)  # 定义不规则多边形的顶点坐标
    points = points.reshape((-1, 1, 2))
    cv.fillPoly(mask, [points], (0, 255, 0, 128))  # 在掩膜上绘制多边形，绿色填充，不透明度为128（半透明）
    mask_img = cv.addWeighted(img, 1, mask, 0.5, 0)  # 将掩膜应用到原图
    mask_img = cv.cvtColor(mask_img, cv.COLOR_BGRA2BGR)
    return mask_img


def is_point_in_polygon(point, polygon):
    """ 判断一个点是否在图像区间，使用射线交叉法（Ray Casting Algorithm）
    - point: 待判断的点，表示为(x, y)的元组; - polygon: 四边形的四个顶点，表示为[(x1, y1), (x2, y2), (x3, y3), (x4, y4)]的列表"""
    x, y = point
    num_polygon = len(polygon)
    num_crossings = 0
    for i in range(num_polygon):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % num_polygon]
        if (y1 > y) != (y2 > y) and x < ((x2 - x1) * (y - y1) / (y2 - y1) + x1):
            num_crossings += 1
    return num_crossings % 2 == 1


def area_pots_sort(p_area, P_area):
    """ 对传入的梯形区域重新排序，确保P_area[0]为左上角，P_area[1]为右上角，P_area[2]为右下角，P_area[3]为左下角   """
    index_list = list(range(len(P_area)))  # 创建一个索引列表，保存原始索引
    indexes = sorted(index_list, key=lambda x: (P_area[x][0], P_area[x][1]))  # 先根据X坐标排序，再根据Y坐标排序，返回索引列表(左下,右下,左上,右上)
    sorted_indexes = [indexes[2], indexes[3], indexes[1], indexes[0]]  # (左上,右上,右下,左下)
    sorted_p_area = [p_area[i] for i in sorted_indexes]  # 根据这些索引来重新排序p_area
    sorted_P_area = [P_area[i] for i in sorted_indexes]  # 根据这些索引来重新排序P_area
    return sorted_p_area, sorted_P_area


def draw_img_axes(img, ymin, ymax, w):
    """ 对传入的图像绘制上坐标轴，其中y轴在图像最底端平行于u轴，其中，ymin:图像最左端，ymax:图像最右端，w:道路宽度，单位都是m   """
    red, green = (255, 0, 0), (0, 0, 255)
    pil_img = Image.fromarray(img)  # 将NumPy数组转换为Pillow图像
    draw = ImageDraw.Draw(pil_img)  # 创建一个可以绘图的对象
    width, height = pil_img.size  # 图像的宽度和高度
    zero_y = round(abs(ymin) / abs(ymax - ymin) * width)  # 计算y轴零点在图像中的位置
    # 绘制水平和垂直中心线
    draw.line((0, height - 1, width - 1, height - 1), fill=green)  # 水平线，颜色为白色
    draw.line((zero_y, 0, zero_y, height), fill=green)  # 垂直线，颜色为白色

    # 绘制坐标轴刻度线和刻度值
    tick_length = 5  # 刻度线长度
    tick_itl = 1   # 刻度间隔, 1m绘制一个
    tick_itl_text = 10  # 横轴刻度值显示间隔, 10刻度间隔显示一个
    font = ImageFont.load_default()

    # 绘制水平轴正半轴刻度线和刻度值
    for y in range(0, abs(ymax) // tick_itl):
        u = zero_y + round(y * tick_itl / abs(ymax - ymin) * width)  # 刻度线所在图像横坐标
        if y % tick_itl_text == 0:
            draw.line((u, height - 1, u, height - 1 - tick_length * 2), fill=red, width=2)
            draw.text((u + 5, height - 1 - tick_length - 10), str(y * tick_itl) + 'm', fill=red, font=font)
        else:
            draw.line((u, height - 1, u, height - 1 - tick_length), fill=green)
    # 绘制水平轴负半轴刻度线和刻度值
    for y in range(1, abs(ymin) // tick_itl):
        u = zero_y - round(y * tick_itl / abs(ymax - ymin) * width)  # 刻度线所在图像横坐标
        if y % tick_itl_text == 0:
            draw.line((u, height - 1, u, height - 1 - tick_length * 2), fill=red, width=2)
            draw.text((u + 5, height - 1 - tick_length - 10), '-' + str(y * tick_itl) + 'm', fill=red, font=font)
        else:
            draw.line((u, height - 1, u, height - 1 - tick_length), fill=green)

    # 绘制垂直轴刻度线和刻度值
    for x in range(1, w):
        v = height - round(x * tick_itl / w * height)  # 刻度线所在图像纵坐标
        draw.line((zero_y, v, zero_y + tick_length, v), fill=green)
        draw.text((zero_y + tick_length + 2, v - 5), str(x)+'m', fill=green, font=font)

    result_img = np.array(pil_img)  # 将图像转换回NumPy数组
    return result_img


def compute_transformation(A, B):
    """ Compute the transformation matrix from point set A to point set B.
        A and B are Nx3 NumPy arrays representing the 3D coordinates of N points. """
    assert A.shape == B.shape and A.shape[1] == 3
    # Compute centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    # Center the point sets
    A_centered = A - centroid_A
    B_centered = B - centroid_B
    # Compute rotation matrix using SVD
    H = np.dot(A_centered.T, B_centered)
    U, S, Vt = svd(H)
    R = np.dot(Vt.T, U.T)
    # Compute translation vector
    t = centroid_B - np.dot(R, centroid_A)
    # Return the transformation matrix and translation vector
    return R, t


if __name__ == '__main__':
    # pt = rot_pt_coordinate(np.pi/2, np.array([0, 0]), [960, 540])
    # print(pt)
    import cv2


