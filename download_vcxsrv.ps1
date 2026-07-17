$url = "https://sourceforge.net/projects/vcxsrv/files/vcxsrv/1.20.9.0/VcXsrv-1.20.9.0-installer.exe/download"
$dest = "D:\VcXsrv\VcXsrv-installer.exe"

Write-Host "Downloading VcXsrv from SourceForge..."
Write-Host "URL: $url"
Write-Host "Dest: $dest"

try {
    # Use WebClient for better compatibility with SourceForge
    $wc = New-Object System.Net.WebClient
    $wc.DownloadFile($url, $dest)
    Write-Host "Download complete!" -ForegroundColor Green
    $size = (Get-Item $dest).Length / 1MB
    Write-Host "Size: $([math]::Round($size, 1)) MB"
} catch {
    Write-Host "Download failed: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "Manual download:"
    Write-Host "1. Open: https://sourceforge.net/projects/vcxsrv/"
    Write-Host "2. Click Download"
    Write-Host "3. Save installer to D:\VcXsrv\"
}
