# UAVs
#自己用动态窗口法写了一个算法，和长机-僚机法结合起来

动态窗口法的原理大致是在速度与加速度约束内，遍历选择所有可能的速度来进行预测未来秒的航迹，通过自己设置的航迹评价函数，例如与目标点距离，与障碍物距离，速度等多方面进行最优选择。

但是与传统优化算法类似，容易陷入局部最优解。表现就是不能避开与目标连线的障碍物。为避免动态窗口法陷入局部最优，提高复杂环境下的航 迹规划能力，在文献的基础上，本文使用一种自适应航 迹评价函数权重的动态窗口法。采用两输入三输出的模糊控 制器优化权重系数，最终我编写代码出来的结果图图
​

​
