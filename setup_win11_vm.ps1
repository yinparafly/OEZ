# ============================================
# Windows 11 Hyper-V 静默安装脚本
# 自动创建 VM + 无人值守安装
# 运行方式: 以管理员 PowerShell 运行
# ============================================

$VMName = "Win11"
$VMPath = "D:\VMs\Win11"
$ISOPath = "D:\ISOs"

Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "Windows 11 静默安装" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan

# 1. 等待 ISO 下载完成
Write-Host "[1] 检查 ISO..." -ForegroundColor Yellow
$ISO = Get-ChildItem -Path $ISOPath -Filter "*.iso" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $ISO) {
    Write-Host "    ISO 未找到, 等待下载..."
    while (-not $ISO) {
        Start-Sleep -Seconds 30
        $ISO = Get-ChildItem -Path $ISOPath -Filter "*.iso" -ErrorAction SilentlyContinue | Select-Object -First 1
        Write-Host "    等待中... $(Get-Date -Format 'HH:mm:ss')"
    }
}
Write-Host "    ISO: $($ISO.Name) ($([math]::Round($ISO.Length/1GB, 2)) GB)" -ForegroundColor Green

# 2. 检查现有 VM
$VM = Get-VM -Name $VMName -ErrorAction SilentlyContinue
if ($VM) {
    Write-Host "[2] VM 已存在, 状态: $($VM.State)" -ForegroundColor Yellow
    if ($VM.State -ne "Running") {
        Start-VM -Name $VMName
        Write-Host "    VM 已启动"
    }
    Write-Host "完成!"
    exit 0
}

# 3. 创建虚拟磁盘
Write-Host "[3] 创建虚拟磁盘 (80GB 动态)..." -ForegroundColor Yellow
$VHDPath = "$VMPath\win11.vhdx"
New-VHD -Path $VHDPath -SizeBytes 80GB -Dynamic
Write-Host "    $VHDPath" -ForegroundColor Green

# 4. 创建 VM
Write-Host "[4] 创建虚拟机..." -ForegroundColor Yellow
New-VM -Name $VMName -MemoryStartupBytes 4GB -Generation 2 -VHDPath $VHDPath -Path $VMPath

# 5. 配置 VM
Write-Host "[5] 配置虚拟机..." -ForegroundColor Yellow
Set-VM -Name $VMName -ProcessorCount 4 -MemoryStartupBytes 4GB -DynamicMemory -MemoryMinimumBytes 2GB -MemoryMaximumBytes 8GB
Set-VM -Name $VMName -CheckpointType Standard

# 网络
$switch = Get-VMSwitch -Name "Default Switch" -ErrorAction SilentlyContinue
if ($switch) {
    Connect-VMNetworkAdapter -VMName $VMName -SwitchName "Default Switch"
    Write-Host "    网络: Default Switch" -ForegroundColor Green
}

# 挂载 ISO
Add-VMDvdDrive -VMName $VMName -Path $ISO.FullName
$dvd = Get-VMDvdDrive -VMName $VMName
Set-VMFirmware -VMName $VMName -FirstBootDevice $dvd
Write-Host "    ISO: $($ISO.Name)" -ForegroundColor Green

# TPM
try {
    Enable-VMTPM -VMName $VMName -ErrorAction Stop
    Write-Host "    TPM: 已启用" -ForegroundColor Green
} catch {
    Write-Host "    TPM: 启用失败, 继续..." -ForegroundColor Yellow
}

# 6. 创建无人值守应答文件
Write-Host "[6] 创建应答文件..." -ForegroundColor Yellow
$Unattend = @"
<?xml version="1.0" encoding="utf-8"?>
<unattend xmlns="urn:schemas-microsoft-com:unattend">
    <settings pass="windowsPE">
        <component name="Microsoft-Windows-International-Core-WinPE" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
            <SetupUILanguage><UILanguage>zh-CN</UILanguage></SetupUILanguage>
            <InputLocale>zh-CN</InputLocale>
            <SystemLocale>zh-CN</SystemLocale>
            <UILanguage>zh-CN</UILanguage>
            <UserLocale>zh-CN</UserLocale>
        </component>
        <component name="Microsoft-Windows-Setup" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
            <DiskConfiguration>
                <Disk wcm:action="add">
                    <CreatePartitions>
                        <CreatePartition wcm:action="add"><Order>1</Order><Type>Primary</Type><Extend>true</Extend></CreatePartition>
                    </CreatePartitions>
                    <ModifyPartitions>
                        <ModifyPartition wcm:action="add"><Order>1</Order><PartitionID>1</PartitionID><Format>NTFS</Format><Label>Windows</Label><Letter>C</Letter></ModifyPartition>
                    </ModifyPartitions>
                    <DiskID>0</DiskID><WillWipeDisk>true</WillWipeDisk>
                </Disk>
            </DiskConfiguration>
            <ImageInstall>
                <OSImage>
                    <InstallFrom><MetaData wcm:action="add"><Key>/IMAGE/NAME</Key><Value>Windows 11 Pro</Value></MetaData></InstallFrom>
                    <InstallTo><DiskID>0</DiskID><PartitionID>1</PartitionID></InstallTo>
                </OSImage>
            </ImageInstall>
        </component>
    </settings>
    <settings pass="oobeSystem">
        <component name="Microsoft-Windows-Shell-Setup" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
            <OOBE><ProtectYourPC>3</ProtectYourPC><NetworkLocation>Home</NetworkLocation></OOBE>
            <UserAccounts>
                <LocalAccounts>
                    <LocalAccount wcm:action="add">
                        <Name>Dev</Name><Group>Administrators</Group>
                        <Password><PlainText>true</PlainText><Value>dev123456</Value></Password>
                    </LocalAccount>
                </LocalAccounts>
            </UserAccounts>
        </component>
    </settings>
</unattend>
"@
$Unattend | Out-File -FilePath "$VMPath\unattend.xml" -Encoding UTF8
Write-Host "    $VMPath\unattend.xml" -ForegroundColor Green

# 7. 启动 VM
Write-Host "[7] 启动虚拟机..." -ForegroundColor Yellow
Start-VM -Name $VMName
Write-Host "    VM 已启动!" -ForegroundColor Green

Write-Host ""
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "安装已开始!" -ForegroundColor Green
Write-Host "查看进度: vmconnect.exe localhost $VMName"
Write-Host "预计 30-50 分钟完成"
Write-Host "=========================================" -ForegroundColor Cyan
