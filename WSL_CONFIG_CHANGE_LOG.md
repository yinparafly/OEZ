# WSL 配置修改日志

## 时间: 2026-07-05 00:25

## 操作: 启用 mirrored 网络模式

### 原始配置 (备份)
```ini
[wsl2]
memory=4GB
processors=4
swap=4GB
localhostForwarding=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

### 新配置
```ini
[wsl2]
memory=4GB
processors=4
swap=4GB
localhostForwarding=true
networkingMode=mirrored
dnsTunneling=true
firewall=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

### 修改内容
- 新增: networkingMode=mirrored (启用镜像网络)
- 新增: dnsTunneling=true (DNS隧道)
- 新增: firewall=true (防火墙)
- 保留: localhostForwarding=true

### 恢复方法
如需恢复，将 .wslconfig 改回原始配置，然后运行:
```powershell
wsl --shutdown
```

### 验证方法
重启后检查:
```bash
# WSL 中检查 IP (应该与 Windows 相同)
ip addr show eth0

# Windows 中测试端口
Test-NetConnection -ComputerName 127.0.0.1 -Port 5760
```
