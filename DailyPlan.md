# 2025-03-19
## YKB's plan
- [ ] [Hector ROS环境](https://github.com/DRCL-USC/Hector_Simulation/tree/ROS_Simulation)
	- [ ] 改步频是否能保持稳定
 	- [ ] 搞懂Unitree_ROS包在其中发挥什么作用
- [ ] [Hector mujoco环境](https://github.com/humarobot/Hector_Simulation)
- [ ] [宇树 ROS环境](https://github.com/unitreerobotics)
- [ ] [宇树RL环境](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/README_zh.md)
- [ ] hector的Casadi单步计算耗时多少测一下，大致了解casadi的python语法？
- [ ] 宇树G1的运动学怎么算，用匹诺曹库？

# 2025-03-20
确定“三步走”战略
1. 将G1模型在导入到Ubuntu22的青龙环境中，实现MPC稳定行走；（任务时间[[2025-03-23]]，任务主要完成人：YKB）
	- [ ] 注意事项1：首先测试它所有的demo，修改步频检查效果
	- [ ] 注意事项2：确认用哪一个g1的模型
	- [ ] 注意事项3：MPC需要运动学推导，要么用Pinocchio库，要么用解析式[Github](https://github.com/junhengl/unitree_g1_optimal_controllers)

2. 二选一（任务时间[[2025-03-30]]，任务主要完成人：MHL&YKB）
	1. 将青龙的C++代码转成python在修改后的deploy_mujoco实现稳定行走；
		- [ ] 注意事项1：首先对整个代码框架进行梳理和任务分解，随后进行分工；
		- [ ] 注意事项2：qpOASES在python上的实时性能如何，进行测试
	2. 将青龙的C++代码封装成可调用的python代码

3. 修改deploy_mujoco，完成实验前序工作（任务时间[[2025-04-01]]，任务主要完成人：YKB）

4. （进阶）训练实现NN-MPC稳定行走

5. （进阶）根据MPC-Net论文对NN-MPC进行相应的修改
