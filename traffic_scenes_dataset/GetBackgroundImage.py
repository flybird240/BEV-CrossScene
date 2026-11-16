# Revised by: ww 2024/2/3


if __name__ == '__main__':
    import cv2
    import os

    # 创建背景减除器MOG2
    backSub = cv2.createBackgroundSubtractorMOG2()

    # 指定视频源，可以是视频文件路径或摄像头ID，或者直接使用数字，例如 0，如果你想从摄像头获取视频流
    video_source = 'E:\\code\\traffic_scenes_dataset\\Shaanxi High Speed Electronics\\K1077+970\\video.mp4'
    current_file_folder = os.path.dirname(video_source)

    # 打开视频文件或摄像头
    capture = cv2.VideoCapture(video_source)

    if not capture.isOpened():
        print("Error opening video stream or file")

    # 设置显示背景图像的帧间隔
    display_interval = 50
    frame_count = 0

    while True:
        ret, frame = capture.read()
        if frame is None:
            break

        # 应用背景减除
        fgMask = backSub.apply(frame)

        # 每隔一定帧数显示背景图像
        if frame_count % display_interval == 0:
            background = backSub.getBackgroundImage()  # 获取背景图像
            if background is not None:
                cv2.imwrite(current_file_folder + '/back_ground.jpg', background)
                cv2.imshow('Background Image', background)
            else:
                print("Background image could not be retrieved.")
        frame_count += 1

        keyboard = cv2.waitKey(30)
        if keyboard == 'q' or keyboard == 27:
            break

    # 释放资源和关闭窗口
    capture.release()
    cv2.destroyAllWindows()
