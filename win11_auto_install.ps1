# ============================================
# Windows 11 一键安装 (下载 + 创建 VM)
# 以管理员 PowerShell 运行
# 后台静默执行, 无需交互
# ============================================

Write-Host "========================================="
Write-Host "Windows 11 一键安装"
Write-Host "========================================="

# 1. 创建目录
New-Item -ItemType Directory -Path "D:\ISOs" -Force | Out-Null
New-Item -ItemType Directory -Path "D:\VMs\Win11" -Force | Out-Null

# 2. 下载 ISO (如果不存在)
$ISO = Get-ChildItem -Path "D:\ISOs" -Filter "*.iso" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $ISO) {
    Write-Host "下载 Windows 11 ISO..."
    $ISOURL = "https://software-static.download.prod.microsoft.com/sg/download/888969d5-f34g-4e03-ac9d-1f9786c66749/22631.2861.231220-0149.23H2_GE2_RELEASE_SVC_EVAL_CLIENTRET_OEMRET_A64FRE_ZH-CN.ISO"
    $ISODest = "D:\ISOs\Win11_23H2.iso"
    Start-BitsTransfer -Source $ISOURL -Destination $ISODest -Description "Windows 11"
    $ISO = Get-Item $ISODest
}

Write-Host "ISO: $($ISO.Name)"

# 3. 创建 VM
$VMName = "Win11"
$VMPath = "D:\VMs\Win11"
$VHDPath = "$VMPath\win11.vhdx"

if (Get-VM -Name $VMName -ErrorAction SilentlyContinue) {
    Write-Host "VM 已存在"
} else {
    New-VHD -Path $VHDPath -SizeBytes 80GB -Dynamic
    New-VM -Name $VMName -MemoryStartupBytes 4GB -Generation 2 -VHDPath $VHDPath -Path $VMPath
    Set-VM -Name $VMName -ProcessorCount 4 -MemoryStartupBytes 4GB -DynamicMemory -MemoryMinimumBytes 2GB -MemoryMaximumBytes 8GB
    Set-VM -Name $VMName -CheckpointType Standard
    Connect-VMNetworkAdapter -VMName $VMName -SwitchName "Default Switch"
    Add-VMDvdDrive -VMName $VMName -Path $ISO.FullName
    Set-VMFirmware -VMName $VMName -FirstBootDevice (Get-VMDvdDrive -VMName $VMName)
    try { Enable-VMTPM -VMName $VMName } catch {}
    Write-Host "VM 已创建"
}

# 4. 无人值守应答文件
@"
<?xml version="1.0" encoding="utf-8"?>
<unattend xmlns="urn:schemas-microsoft-com:unattend">
<settings pass="windowsPE">
<component name="Microsoft-Windows-International-Core-WinPE" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
<SetupUILanguage><UILanguage>zh-CN</UILanguage></SetupUILanguage>
<InputLocale>zh-CN</InputLocale><SystemLocale>zh-CN</SystemLocale><UILanguage>zh-CN</UILanguage><UserLocale>zh-CN</UserLocale>
</component>
<component name="Microsoft-Windows-Setup" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
<DiskConfiguration><Disk wcm:action="add"><CreatePartitions><CreatePartition wcm:action="add"><Order>1</Order><Type>Primary</Type><Extend>true</Extend></CreatePartition></CreatePartitions><ModifyPartitions><ModifyPartition wcm:action="add"><Order>1</Order><PartitionID>1</PartitionID><Format>NTFS</Format><Label>Windows</Label><Letter>C</Letter></ModifyPartition></ModifyPartitions><DiskID>0</DiskID><WillWipeDisk>true</WillWipeDisk></Disk></DiskConfiguration>
<ImageInstall><OSImage><InstallFrom><MetaData wcm:action="add"><Key>/IMAGE/NAME</Key><Value>Windows 11 Pro</Value></MetaData></InstallFrom><InstallTo><DiskID>0</DiskID><PartitionID>1</PartitionID></InstallTo></OSImage></ImageInstall>
</component>
</settings>
<settings pass="oobeSystem">
<component name="Microsoft-Windows-Shell-Setup" processorArchitecture="amd64" publicKeyToken="31bf3856ad364e35" language="neutral" versionScope="nonSxS">
<OOBE><ProtectYourPC>3</ProtectYourPC><NetworkLocation>Home</NetworkLocation></OOBE>
<UserAccounts><LocalAccounts><LocalAccount wcm:action="add"><Name>Dev</Name><Group>Administrators</Group><Password><PlainText>true</PlainText><Value>dev123456</Value></Password></LocalAccount></LocalAccounts></UserAccounts>
</component>
</settings>
</unattend>
"@ | Out-File -FilePath "$VMPath\unattend.xml" -Encoding UTF8

# 5. 启动
Start-VM -Name $VMName
Write-Host "VM 已启动! 安装约 30-50 分钟"
Write-Host "查看: vmconnect.exe localhost $VMName"
