# Windows 11 ISO 静默下载脚本
$ISOPath = "D:\ISOs\Win11_23H2_Chinese_Simplified_x64.iso"
$URL = "https://software-static.download.prod.db.microsoft.com/sg/download/888969d5-f34g-4e03-ac9d-1f9786c66749/22631.2861.231220-0149.23H2_GE2_RELEASE_SVC_EVAL_CLIENTRET_OEMRET_A64FRE_ZH-CN.ISO"

Write-Host "========================================="
Write-Host "下载 Windows 11 ISO"
Write-Host "目标: $ISOPath"
Write-Host "========================================="

if (Test-Path $ISOPath) {
    Write-Host "ISO 已存在, 跳过下载"
} else {
    Write-Host "开始下载 (约 5.2GB, 需要 10-20 分钟)..."
    try {
        # 使用 BITS 后台下载
        Start-BitsTransfer -Source $URL -Destination $ISOPath -Description "Windows 11 ISO" -ErrorAction Stop
        Write-Host "下载完成!"
    } catch {
        Write-Host "BITS 下载失败, 尝试 Invoke-WebRequest..."
        try {
            Invoke-WebRequest -Uri $URL -OutFile $ISOPath -UseBasicParsing
            Write-Host "下载完成!"
        } catch {
            Write-Host "下载失败: $_"
            Write-Host ""
            Write-Host "请手动下载 Windows 11 ISO:"
            Write-Host "https://www.microsoft.com/software-download/windows11"
            Write-Host "保存到: D:\ISOs\"
        }
    }
}

# 检查下载结果
if (Test-Path $ISOPath) {
    $size = (Get-Item $ISOPath).Length / 1GB
    Write-Host "ISO 大小: $([math]::Round($size, 2)) GB"
    Write-Host "下载成功!"
} else {
    Write-Host "ISO 不存在"
}
