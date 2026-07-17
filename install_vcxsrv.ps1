# VcXsrv 安装到 D 盘
$InstallDir = "D:\VcXsrv"
New-Item -ItemType Directory -Path $InstallDir -Force | Out-Null

Write-Host "========================================="
Write-Host "Installing VcXsrv to D:\VcXsrv"
Write-Host "========================================="

# 下载 VcXsrv
$url = "https://sourceforge.net/projects/vcxsrv/files/vcxsrv/1.20.9.0/VcXsrv-1.20.9.0-installer.exe/download"
$installer = "$InstallDir\VcXsrv-installer.exe"

Write-Host "Downloading VcXsrv..."
try {
    Invoke-WebRequest -Uri $url -OutFile $installer -UseBasicParsing -ErrorAction Stop
    Write-Host "Download complete!" -ForegroundColor Green
} catch {
    Write-Host "Download failed: $_" -ForegroundColor Red
    Write-Host "Please download manually from: https://sourceforge.net/projects/vcxsrv/"
    Write-Host "Save to: $installer"
    exit 1
}

# 静默安装
Write-Host "Installing VcXsrv..."
Start-Process -FilePath $installer -ArgumentList "/S" -Wait -ErrorAction SilentlyContinue

# 检查安装
$exe = "C:\Program Files (x86)\XWin Server\XWin Server.exe"
if (Test-Path $exe) {
    Write-Host "VcXsrv installed successfully!" -ForegroundColor Green
    Write-Host "Location: $exe"
} else {
    # 尝试其他路径
    $exe2 = "C:\Program Files\VcXsrv\XWin Server.exe"
    if (Test-Path $exe2) {
        Write-Host "VcXsrv installed!" -ForegroundColor Green
    } else {
        Write-Host "Installation may need manual verification" -ForegroundColor Yellow
    }
}
