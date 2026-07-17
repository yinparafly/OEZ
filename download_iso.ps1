New-Item -ItemType Directory -Path "D:\ISOs" -Force | Out-Null
Write-Host "Downloading Windows 11 ISO..."
$url = "https://software-static.download.prod.microsoft.com/sg/download/888969d5-f34g-4e03-ac9d-1f9786c66749/22631.2861.231220-0149.23H2_GE2_RELEASE_SVC_EVAL_CLIENTRET_OEMRET_A64FRE_ZH-CN.ISO"
$dest = "D:\ISOs\Win11_23H2.iso"
try {
    Invoke-WebRequest -Uri $url -OutFile $dest -UseBasicParsing
    Write-Host "Download complete!"
    $size = (Get-Item $dest).Length / 1GB
    Write-Host "Size: $([math]::Round($size, 2)) GB"
} catch {
    Write-Host "Download failed: $_"
}
