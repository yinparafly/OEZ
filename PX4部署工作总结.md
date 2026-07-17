# PX4 开发环境部署 — 工作总结

**日期：** 2026-07-02 ~ 2026-07-03
**目标：** 在 Windows 机器上通过 WSL2 搭建 PX4 开发环境，编译 Radiolink PIX6 飞控固件

---

## 一、已完成的工作

### 1. WSL2 环境搭建
- 启用 Windows 功能：Microsoft-Windows-Subsystem-Linux、VirtualMachinePlatform
- 设置 WSL2 为默认版本
- 重启电脑使功能生效
- 手动下载 Ubuntu 22.04 rootfs（从 Ubuntu 官网，28MB）
- 通过 `wsl --import` 导入到 `D:\pix\wsl\Ubuntu-22.04\`
- 配置 `C:\Users\tt\.wslconfig`（4GB 内存、4 核 CPU、4GB 交换）

### 2. 编译工具链安装
- 基础工具：cmake 3.22.1、ninja 1.10.1、gcc/g++ 11、git 2.34.1、Python 3.10.12
- ARM 交叉编译器：gcc-arm-none-eabi 10.3.1
- Python 依赖（11 个包）：numpy、pandas、pyyaml、cerberus、packaging、jinja2、jsonschema、pyulog、pyros-genmsg、kconfiglib、empy
- pip 使用清华镜像源加速（`https://pypi.tuna.tsinghua.edu.cn/simple`）

### 3. 网络/代理配置
- Windows 代理：127.0.0.1:7890（Clash）
- WSL2 内访问 Windows 代理：172.30.16.1:7890
- 端口转发：`netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=7890 connectaddress=127.0.0.1 connectport=7890`（需管理员权限）
- Git 代理已配置：`http.proxy=http://172.30.16.1:7890`、`https.proxy=http://172.30.16.1:7890`

### 4. PX4 源码获取
- 通过 GitHub 代理成功 clone PX4-Autopilot v1.17.0
- 全部 29 个子模块已初始化（NuttX、MAVLink、GPS 驱动等）
- 源码位置：WSL 内 `/root/px4`（原生 ext4 文件系统，编译快）+ Windows `D:\pix\px4`

### 5. 固件编译成功
- 目标板：radiolink_PIX6
- 编译命令：`make radiolink_PIX6`
- FLASH 使用率：94.38%
- 产物：
  - `D:\pix\radiolink_PIX6_default.px4`（1.72MB）
  - `D:\pix\radiolink_PIX6_default.bin`（1.86MB）

---

## 二、遇到的问题及解决方案

### 问题 1：Microsoft Store 下载 Ubuntu 极慢（0x80072efd 错误）
- **原因：** 网络无法连接 Microsoft Store 服务器
- **解决：** 改为从 Ubuntu 官网直接下载 rootfs tar.gz（28MB），然后 `wsl --import` 手动导入

### 问题 2：WSL2 内无法访问 GitHub
- **原因：** WSL2 是独立网络，`127.0.0.1:7890` 指向 WSL 自己而非 Windows
- **解决：** 三步：(1) 获取 Windows 宿主机 IP（`/etc/resolv.conf` 中的 nameserver=172.30.16.1）；(2) 管理员权限运行 `netsh portproxy` 把 7890 端口转发到 0.0.0.0；(3) git 配置全局代理

### 问题 3：pip 安装 Python 包极慢
- **解决：** 切换 pip 镜像源到清华：`pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple`

### 问题 4：ZIP 下载的 PX4 源码 Makefile 拒绝编译
- **错误：** `YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING..`
- **解决：** 不用 ZIP，改用 git clone（需代理）

### 问题 5：WSL 通过 /mnt/d 编译极慢
- **原因：** WSL 访问 Windows NTFS 文件系统 I/O 开销极大，cmake 配置 20 分钟跑不完
- **解决：** 用 `tar cf - px4 | tar xf - -C /root/` 把源码复制到 WSL 原生 ext4 文件系统，在 `/root/px4` 下编译

### 问题 6：px4_fmu-v5x 目标编译 FLASH 溢出
- **错误：** `FLASH_AXIM overflowed by 4529 bytes`
- **解决：** 改用正确的板级目标 `radiolink_PIX6`（该配置针对具体硬件裁剪了模块）

### 问题 7：缺少 Python 模块 empy 和 kconfiglib
- **解决：** `pip3 install empy==3.3.4 kconfiglib`

### 问题 8：子模块不完整（heatshrink 目录为空）
- **原因：** tar 复制未完整传递 git submodule 工作树
- **解决：** `git submodule update --force --init --recursive`

---

## 三、可直接拷贝到另一台计算机的文件

| 文件/目录 | 源路径 | 用途 | 大小 |
|-----------|--------|------|------|
| `PX4-Autopilot-1.17.0.zip` | `D:\pix\PX4-Autopilot-1.17.0.zip` | PX4 源码包（备用） | 275MB |
| `radiolink_PIX6_default.px4` | `D:\pix\radiolink_PIX6_default.px4` | 编译好的固件，可直接烧录 | 1.72MB |
| `radiolink_PIX6_default.bin` | `D:\pix\radiolink_PIX6_default.bin` | 固件二进制格式 | 1.86MB |
| `ubuntu-22.04-rootfs.tar.gz` | 临时目录 | Ubuntu rootfs（可重用导入） | 28MB |
| `.wslconfig` | `C:\Users\tt\.wslconfig` | WSL2 配置（内存/核数/交换） | <1KB |
| `px4-env-setup.md` | `D:\oezcon\px4-env-setup.md` | 部署记录 | - |
| PX4 模块分析 | `D:\pix\PX4_v1.17.0_模块分析.md` | 技术参考 | - |

**不可拷贝但可重新生成的：**
- WSL 内的 Ubuntu 系统（重新 import rootfs 即可）
- WSL 内安装的编译工具链（重新 apt install 即可）
- PX4 源码和子模块（重新 git clone 即可）

---

## 四、待完成工作

1. ~~搭建 WSL2 环境~~ ✅
2. ~~安装编译工具链~~ ✅
3. ~~获取 PX4 源码~~ ✅
4. ~~编译 Radiolink PIX6 固件~~ ✅
5. **气球投放飞行方案设计** — 待分析
6. **PX4 程序修改（投放拉起逻辑）** — 待分析
7. **QGroundControl 烧录固件** — 需用户操作
8. **首飞前参数校准** — 需用户操作
9. **Gazebo 仿真测试** — 可选

---

## 五、快速复现命令（在新机器上）

```bash
# 1. 启用 WSL（PowerShell 管理员）
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
# 重启后：
wsl --set-default-version 2

# 2. 导入 Ubuntu
wsl --import Ubuntu-22.04 D:\pix\wsl\Ubuntu-22.04 <rootfs.tar.gz路径> --version 2

# 3. 配置代理（进入 WSL 后）
git config --global http.proxy http://<WindowsIP>:7890
git config --global https.proxy http://<WindowsIP>:7890
pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# 4. 安装工具链
apt update && apt install -y build-essential cmake ninja-build genromfs \
  python3-pip python3-venv lsb-release software-properties-common \
  gcc-arm-none-eabi git curl wget gnupg2
pip3 install numpy pandas pyyaml cerberus packaging jinja2 jsonschema \
  pyulog pyros-genmsg kconfiglib empy==3.3.4

# 5. 克隆 PX4
cd /root && git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b v1.17.0 px4

# 6. 编译
cd /root/px4 && make radiolink_PIX6
```
