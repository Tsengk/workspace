编译：
    在当前目录下打开终端执行：catkin_make

配置参数：
    将front_h60_intrinsics.yaml放入config文件下

运行：
    1.source devel/setup.bash
    2.roslaunch camera_tuning_suite tuning_system.launch mode:=real(mode:=sim)

保存：
    在调参窗口键盘输入s，可在终端输出参数结果，需手动输入到配置文件中保存

参数说明：
    内参矩阵 (Camera Matrix K)：决定“投影几何”
        f_x, f_y (Focal Length / 焦距)：相机光心到成像平面（传感器）的距离，单位是像素(pixels)
            变大：图像放大 (Zoom In)，视场角 (FOV) 变窄，物体变大
            变小：图像缩小 (Zoom Out)，视场角变宽，物体变小。
        c_x, c_y (Principal Point / 主点)：光轴（Optical Axis）与成像平面的交点，通常接近图像的几何中心 (Width/2, Height/2)
            调节 c_x：整幅图像左右平移
            调节 c_y：整幅图像上下平移
    畸变系数 (Distortion Coefficients D)：决定“形状矫正”
        D = [k_1, k_2, p_1, p_2, k_3]
        k_1, k_2, k_3 (Radial Distortion / 径向畸变)：光线穿过透镜边缘时弯曲程度不同。越靠近边缘，弯曲越严重
            k_1 (主项)：
                负值 (k_1 < 0)：修正桶形畸变 (Barrel Distortion)。常见于广角/鱼眼镜头（如 GMSL 车载相机）。如果画面像气球一样鼓起来，需要减小 k_1。
                正值 (k_1 > 0)：修正枕形畸变 (Pincushion Distortion)。常见于长焦镜头。如果画面像枕头一样向内凹陷，需要增大 k_1。
            k_2, k_3 (高阶项)：
                用于微调图像最边缘/四角的弯曲。通常先调好 k_1 把中间大部分变直，再动 k_2 把角落拉直。
        p_1, p_2 (Tangential Distortion / 切向畸变)：由于透镜安装不完美，导致透镜平面与传感器平面不平行（有倾角）
            p_1：修正看起来像“一边近、一边远”的水平倾斜
            p_2：修正垂直倾斜
    矩阵（Projection Matrix P）是“计算结果”：在终端查看
    视场控制 (Alpha / Zoom)
        Alpha = 0 (Zoom: 0%)：最大化裁剪
        Alpha = 1 (Zoom: 100%)：保留所有原图像素
