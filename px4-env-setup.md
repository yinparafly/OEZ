# PX4 开发环境搭建 - 工作记录

## 目标
在 Windows 上通过 WSL2 搭建 PX4 开发环境，所有内容安装在 `D:\pix` 目录下，使用 Ubuntu 22.04 LTS。

## 进度

### 已完成
- [x] 启用 WSL 功能 (Microsoft-Windows-Subsystem-Linux)
- [x] 启用虚拟机平台 (VirtualMachinePlatform)
- [x] 设置 WSL2 为默认版本

### 已完成（全部）
- [x] 安装 Ubuntu 22.04 发行版（通过手动下载 rootfs 导入 WSL）
- [x] 创建 D:\pix 目录结构
- [x] 将 WSL 数据（ext4.vhdx）存放在 D:\pix\wsl\
- [x] 配置 .wslconfig 指向 D 盘（4GB内存/4核/4GB交换）
- [x] 安装 PX4 开发工具链：
  - [x] 基础编译工具：cmake 3.22.1, ninja 1.10.1, gcc/g++ 11
  - [x] ARM 交叉编译器：gcc-arm-none-eabi 10.3.1
  - [x] Python 依赖：numpy, pandas, pyyaml, cerberus, packaging, jinja2, jsonschema, pyulog, pyros-genmsg, kconfiglib, empy
  - [x] PX4 源码：v1.17.0（git clone + 全部子模块）
  - [x] NuttX 相关依赖
- [x] WSL 代理配置（172.30.16.1:7890）
- [x] PX4 固件编译成功：radiolink_PIX6_default.px4
- [x] 固件已拷贝至 D:\pix\

## 关键路径
- WSL 安装目录：`D:\pix\wsl\Ubuntu-22.04\`
- PX4 源码目录（D盘）：`D:\pix\px4\`
- PX4 源码目录（WSL原生，编译用）：`~/px4`（/root/px4）
- 编译产出：`D:\pix\radiolink_PIX6_default.px4`
- WSL 配置文件：`%USERPROFILE%\.wslconfig`
- WSL 代理：`http://172.30.16.1:7890`（git 已配置）

## 环境信息
- 操作系统：Windows
- WSL 版本：WSL2
- 发行版：Ubuntu 22.04 LTS
- 安装目标盘：D:\pix

## 备注
- WSL 功能启用后必须重启电脑才能生效
- 重启后回来继续执行剩余步骤
