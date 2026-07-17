# ============================================
# Windows 11 Hyper-V 无人值守安装脚本
# 运行方式: 以管理员身份运行 PowerShell
# 预计时间: 50-70 分钟 (全自动, 无需交互)
# ============================================

# 1. 创建目录
$VMPath = "D:\VMs\Win11"
$ISOPath = "D:\ISOs"
New-Item -ItemType Directory -Path $VMPath -Force | Out-Null
New-Item -ItemType Directory -Path $ISOPath -Force | Out-Null

Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "Windows 11 Hyper-V 自动安装" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host ""

# 2. 检查 Windows 11 ISO
$Win11ISO = Get-ChildItem -Path $ISOPath -Filter "Win11*.iso" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $Win11ISO) {
    Write-Host "[1] 下载 Windows 11 ISO..." -ForegroundColor Yellow
    Write-Host "    请手动下载: https://www.microsoft.com/software-download/windows11"
    Write-Host "    选择: Windows 11 Disk Image (ISO) for x64 devices"
    Write-Host "    保存到: $ISOPath"
    Write-Host ""
    Write-Host "    下载完成后重新运行此脚本"
    exit 1
}
Write-Host "[1] ISO 已找到: $($Win11ISO.Name)" -ForegroundColor Green

# 3. 检查现有虚拟机
$VM = Get-VM -Name "Win11" -ErrorAction SilentlyContinue
if ($VM) {
    Write-Host "[2] 虚拟机 'Win11' 已存在, 状态: $($VM.State)" -ForegroundColor Yellow
    if ($VM.State -eq "Running") {
        Write-Host "    虚拟机正在运行, 跳过创建"
    } else {
        Write-Host "    启动虚拟机..."
        Start-VM -Name "Win11"
    }
} else {
    Write-Host "[2] 创建虚拟机..." -ForegroundColor Yellow

    # 创建虚拟磁盘
    $VHDPath = "$VMPath\win11.vhdx"
    if (-not (Test-Path $VHDPath)) {
        New-VHD -Path $VHDPath -SizeBytes 80GB -Dynamic
        Write-Host "    虚拟磁盘已创建: $VHDPath"
    }

    # 创建虚拟机
    New-VM -Name "Win11" -MemoryStartupBytes 4GB -Generation 2 -VHDPath $VHDPath -Path $VMPath

    # 配置
    Set-VM -Name "Win11" -ProcessorCount 4 -MemoryStartupBytes 4GB -DynamicMemory -MemoryMinimumBytes 2GB -MemoryMaximumBytes 8GB
    Set-VM -Name "Win11" -CheckpointType Standard

    # 网络
    $switch = Get-VMSwitch -Name "Default Switch" -ErrorAction SilentlyContinue
    if ($switch) {
        Connect-VMNetworkAdapter -VMName "Win11" -SwitchName "Default Switch"
    }

    # 挂载 ISO
    Add-VMDvdDrive -VMName "Win11" -Path $Win11ISO.FullName
    $dvd = Get-VMDvdDrive -VMName "Win11"
    Set-VMFirmware -VMName "Win11" -FirstBootDevice $dvd

    # 启用 TPM (Windows 11 需要)
    try {
        Enable-VMTPM -VMName "Win11"
        Write-Host "    TPM 已启用" -ForegroundColor Green
    } catch {
        Write-Host "    TPM 启用失败 (可能不支持), 继续安装..." -ForegroundColor Yellow
    }

    Write-Host "    虚拟机已创建" -ForegroundColor Green
}

# 4. 创建无人值守应答文件
Write-Host "[3] 创建无人值守应答文件..." -ForegroundColor Yellow
$UnattendContent = @"
<?xml version="1.0" encoding="utf-8"?>
<unattend xmlns="urn:schemas-microsoft-com:unattend">
    <settings pass="windowsPE">
        <component name="Microsoft-Windows-International-Core-WinPE" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
            <SetupUILanguage>
                <UILanguage>zh-CN</UILanguage>
            </SetupUILanguage>
            <InputLocale>zh-CN</InputLocale>
            <SystemLocale>zh-CN</SystemLocale>
            <UILanguage>zh-CN</UILanguage>
            <UserLocale>zh-CN</UserLocale>
        </component>
        <component name="Microsoft-Windows-Setup" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
            <DiskConfiguration>
                <Disk wcm:action="add">
                    <CreatePartitions>
                        <CreatePartition wcm:action="add">
                            <Order>1</Order>
                            <Type>Primary</Type>
                            <Extend>true</Extend>
                        </CreatePartition>
                    </CreatePartitions>
                    <ModifyPartitions>
                        <ModifyPartition wcm:action="add">
                            <Order>1</Order>
                            <PartitionID>1</PartitionID>
                            <Format>NTFS</Format>
                            <Label>Windows</Label>
                            <Letter>C</Letter>
                        </ModifyPartition>
                    </ModifyPartitions>
                    <DiskID>0</DiskID>
                    <WillWipeDisk>true</WillWipeDisk>
                </Disk>
            </DiskConfiguration>
            <ImageInstall>
                <OSImage>
                    <InstallFrom>
                        <MetaData wcm:action="add">
                            <Key>/IMAGE/NAME</Key>
                            <Value>Windows 11 Pro</Value>
                        </MetaData>
                    </InstallFrom>
                    <InstallTo>
                        <DiskID>0</DiskID>
                        <PartitionID>1</PartitionID>
                    </InstallTo>
                </OSImage>
            </ImageInstall>
        </component>
    </settings>
    <settings pass="oobeSystem">
        <component name="Microsoft-Windows-Shell-Setup" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
            <OOBE>
                <ProtectYourPC>3</ProtectYourPC>
                <NetworkLocation>Home</NetworkLocation>
            </OOBE>
            <UserAccounts>
                <LocalAccounts>
                    <LocalAccount wcm:action="add">
                        <Name>Dev</Name>
                        <Group>Administrators</Group>
                        <Password>
                            <PlainText>true</PlainText>
                            <Value>dev123456</Value>
                        </Password>
                    </LocalAccount>
                </LocalAccounts>
            </UserAccounts>
        </component>
    </settings>
</unattend>
"@

$UnattendPath = "$VMPath\unattend.xml"
$UnattendContent | Out-File -FilePath $UnattendPath -Encoding UTF8
Write-Host "    应答文件已创建: $UnattendPath" -ForegroundColor Green

# 5. 启动虚拟机
Write-Host "[4] 启动虚拟机..." -ForegroundColor Yellow
$VM = Get-VM -Name "Win11"
if ($VM.State -ne "Running") {
    Start-VM -Name "Win11"
}
Write-Host "    虚拟机已启动" -ForegroundColor Green

# 6. 等待安装完成
Write-Host "[5] 等待 Windows 安装..." -ForegroundColor Yellow
Write-Host "    (大约需要 30-50 分钟, 可以去做其他事情)"
Write-Host "    用以下命令查看进度:"
Write-Host "    Get-VM -Name 'Win11' | Select-Object State, Uptime"
Write-Host ""
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "安装已在后台开始!" -ForegroundColor Green
Write-Host "VM Connect 连接查看: vmconnect.exe localhost Win11"
Write-Host "=========================================" -ForegroundColor Cyan
