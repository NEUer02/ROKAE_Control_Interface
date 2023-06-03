# 快速开始

## 编译
- cd rci_client
- mkdir -p build 
- cd build
- rm -rf *
- cmake .. && make

## 配置网络

- 编辑xmate.ini文件中的IP和端口号，分别设置成机器人的IP和1337

## 运行程序
- cd build/examples
- sudo ./${example}