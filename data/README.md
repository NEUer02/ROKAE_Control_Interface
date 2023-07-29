# 关于Data文件夹的必要说明
本文件夹下存放了Fourier级数的参数文件和记录关节力矩的文件夹。

激励轨迹与Fourier级数参数的优化请见[此仓库](https://github.com/NEUer02/Dynamic_Parameter_Identification_for_Rokae_xMate)。

## optimal_trajectory
本文件夹下包含5阶傅里叶级数的参数。

5阶傅里叶轨迹计算公式为：
$$
q(t) = bias + \sum^{5}_{i = 1} \frac{a_i}{\omega \times i} \sin(\omega \times i \times t)-\sum^{5}_{i = 1} \frac{b_i}{\omega \times i} \cos(\omega \times i \times t)
$$

其中：
$$
\omega = 0.1 \times \pi
$$
一共七个轴，每个轴的5阶傅里叶级数包含11个参数，为：
$$
a_1, b_1, a_2, b_2, a_3, b_3, a_4, b_4, a_5, b_5, bias
$$

‘opt_x.txt’将每个轴的参数组合为11行1列的向量，并叠加在一起。



## joint_force_log

记录关节力矩的文件是以采集时间命名的。

有两个文件，一个文件格式为txt，一个为csv。两个文件中的信息一致。

文件中记录了帧数、关节力矩、关节位置、关节速度和关节加速度。

PS：csv文件用Excel打开可能会乱码，请使用vscode或其他软件打开。
