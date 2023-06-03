# libRCI_client: C++ library for xMate research

libRCI_client 是xMate机器人外部控制接口，用于通过外部通信的方式控制机器人。
---

[![Platform](https://img.shields.io/badge/platform-%20%20%20%20Linux-green.svg?style=flat)]()

[![Porject Status](https://img.shields.io/badge/Project%20Status-%20Active%20%20-yellow.svg?style=flat)]()

[![Version](https://img.shields.io/badge/Version-%20%20v0.3.0%20-yellow.svg?style=flat)]()

### 依赖的三方库

- boost

- eigen

- orocos-kdl

## windows下编译步骤
- 解压SDK包
- 打开example下的vs工程，编译运行

## Linux下编译步骤
- 解压SDK包
- mkdir -p build 
- cd build
- cmake .. 
- make

## thanks
感谢gtest/libfranka/boost/kdl等项目
