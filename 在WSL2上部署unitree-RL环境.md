1. 首先安装WSL的Unbuntu-20.04
	wsl相关命令[CSDN](https://blog.csdn.net/Go_ahead_forever/article/details/135439924)
	
2. 安装cuda 12.1 (与pytorch 2.3.1 版本对应)
	[Nvidia](https://developer.nvidia.com/cuda-12-1-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local)
	！千万不要在WSL安装nvidia驱动
	*我的 nvcc --version 无返回，但好像并不影响后续使用*
	
3. 安装unitree相关配置文档
	[unitree](https://github.com/unitreerobotics/unitree_rl_gym/blob/main/doc/setup_zh.md)
	
	当装好python后切换镜像源（不镜像下载python包太慢）
	pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
	[清华镜像源](https://mirrors.tuna.tsinghua.edu.cn/help/pypi/)
	
	安装isaacgym文件在windows下的文件夹（E:/Downlowds/isaac*）找
	mnt/e是在wsl下查看windows的e盘
	解压指令：tar  -zxvf *
	
	参照该[blog](https://blog.csdn.net/wsygbthhhh/article/details/143918730)解决显示问题
	
	