# Windows 11 VM Install - ASCII version
$VMName = "Win11"
$VMPath = "D:\VMs\Win11"

Write-Host "========================================="
Write-Host "Windows 11 VM Auto Install"
Write-Host "========================================="

# Check ISO
$ISO = Get-ChildItem -Path "D:\ISOs" -Filter "*.iso" -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $ISO) {
    Write-Host "No ISO found in D:\ISOs"
    exit 1
}
Write-Host "ISO: $($ISO.Name) ($([math]::Round($ISO.Length/1GB, 2)) GB)"

# Check existing VM
$VM = Get-VM -Name $VMName -ErrorAction SilentlyContinue
if ($VM) {
    Write-Host "VM exists. State: $($VM.State)"
    if ($VM.State -ne "Running") { Start-VM -Name $VMName }
    exit 0
}

# Create VHD
$VHDPath = "$VMPath\win11.vhdx"
Write-Host "Creating VHD (80GB)..."
New-VHD -Path $VHDPath -SizeBytes 80GB -Dynamic

# Create VM
Write-Host "Creating VM..."
New-VM -Name $VMName -MemoryStartupBytes 4GB -Generation 2 -VHDPath $VHDPath -Path $VMPath
Set-VM -Name $VMName -ProcessorCount 4 -MemoryStartupBytes 4GB -DynamicMemory -MemoryMinimumBytes 2GB -MemoryMaximumBytes 8GB
Set-VM -Name $VMName -CheckpointType Standard

# Network
$switch = Get-VMSwitch -Name "Default Switch" -ErrorAction SilentlyContinue
if ($switch) {
    Connect-VMNetworkAdapter -VMName $VMName -SwitchName "Default Switch"
    Write-Host "Network: Default Switch"
}

# DVD
Add-VMDvdDrive -VMName $VMName -Path $ISO.FullName
$dvd = Get-VMDvdDrive -VMName $VMName
Set-VMFirmware -VMName $VMName -FirstBootDevice $dvd
Write-Host "DVD: $($ISO.Name)"

# TPM
try {
    Enable-VMTPM -VMName $VMName -ErrorAction Stop
    Write-Host "TPM: Enabled"
} catch {
    Write-Host "TPM: Skipped (not supported)"
}

# Unattend file
$Unattend = @"
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
"@
$Unattend | Out-File -FilePath "$VMPath\unattend.xml" -Encoding UTF8
Write-Host "Unattend file created"

# Start VM
Write-Host "Starting VM..."
Start-VM -Name $VMName

Write-Host ""
Write-Host "========================================="
Write-Host "Installation started!"
Write-Host "View: vmconnect.exe localhost $VMName"
Write-Host "Takes 30-50 minutes"
Write-Host "========================================="
