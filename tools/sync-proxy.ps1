<#
    sync-proxy.ps1

    Reads the Windows system proxy (the one Chrome uses) and applies it to:
      - Current PowerShell session env vars (HTTP_PROXY / HTTPS_PROXY / ALL_PROXY / NO_PROXY)
      - User-level persistent env vars (optional, with -Persist)
      - git global config (http.proxy / https.proxy)
      - npm config (proxy / https-proxy), if npm is installed
      - pip config (global.proxy), if pip is installed

    Usage:
      powershell -ExecutionPolicy Bypass -File tools\sync-proxy.ps1            # session + git/npm/pip
      powershell -ExecutionPolicy Bypass -File tools\sync-proxy.ps1 -Persist   # also write user-level persistent env vars
      powershell -ExecutionPolicy Bypass -File tools\sync-proxy.ps1 -Clear     # clear all proxy settings
#>

param(
    [switch]$Persist,
    [switch]$Clear
)

$ErrorActionPreference = 'Stop'
$regPath = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings'

function Clear-Proxy {
    Write-Host '[*] Clearing proxy settings...' -ForegroundColor Yellow
    foreach ($v in 'HTTP_PROXY','HTTPS_PROXY','ALL_PROXY','http_proxy','https_proxy','all_proxy') {
        Remove-Item "Env:$v" -ErrorAction SilentlyContinue
        if ($Persist) { [Environment]::SetEnvironmentVariable($v, $null, 'User') }
    }
    git config --global --unset http.proxy  2>$null
    git config --global --unset https.proxy 2>$null
    if (Get-Command npm -ErrorAction SilentlyContinue) {
        npm config delete proxy       2>$null
        npm config delete https-proxy 2>$null
    }
    if (Get-Command pip -ErrorAction SilentlyContinue) {
        pip config unset global.proxy 2>$null
    }
    Write-Host '[+] Cleared.' -ForegroundColor Green
}

if ($Clear) { Clear-Proxy; return }

# --- Read system proxy ---
$s = Get-ItemProperty -Path $regPath
$proxyEnable = [int]$s.ProxyEnable
$proxyServer = $s.ProxyServer
$autoConfig  = $s.AutoConfigURL

Write-Host "[*] System proxy: ProxyEnable=$proxyEnable" -ForegroundColor Cyan
Write-Host "[*] ProxyServer  = $proxyServer" -ForegroundColor Cyan
Write-Host "[*] AutoConfigURL= $autoConfig"  -ForegroundColor Cyan

if ($proxyEnable -ne 1 -or [string]::IsNullOrWhiteSpace($proxyServer)) {
    Write-Host ''
    Write-Host '[!] No manual proxy enabled (or a PAC auto-config script is used).' -ForegroundColor Yellow
    if (-not [string]::IsNullOrWhiteSpace($autoConfig)) {
        Write-Host "    PAC script detected: $autoConfig" -ForegroundColor Yellow
        Write-Host '    CLI tools cannot use PAC directly. Switch Chrome/system to a manual proxy, or tell me the host:port.' -ForegroundColor Yellow
    }
    Write-Host '    No changes made.' -ForegroundColor Yellow
    return
}

# ProxyServer may be "host:port" or "http=host:port;https=host:port;..."
function Get-ProxyFor([string]$scheme, [string]$raw) {
    if ($raw -match "(?:^|;)\s*$scheme=([^;]+)") { return $Matches[1].Trim() }
    if ($raw -notmatch '=') { return $raw.Trim() }   # single host:port form, applies to all schemes
    return $null
}

$httpHostPort  = Get-ProxyFor 'http'  $proxyServer
$httpsHostPort = Get-ProxyFor 'https' $proxyServer
if (-not $httpsHostPort) { $httpsHostPort = $httpHostPort }
if (-not $httpHostPort)  { $httpHostPort  = $httpsHostPort }

$httpProxy  = "http://$httpHostPort"
$httpsProxy = "http://$httpsHostPort"

# Bypass local addresses
$noProxy = 'localhost,127.0.0.1,::1'
if (-not [string]::IsNullOrWhiteSpace($s.ProxyOverride)) {
    $ov = ($s.ProxyOverride -replace '<local>','localhost,127.0.0.1,::1') -replace ';',','
    $noProxy = $ov
}

Write-Host ''
Write-Host "[*] Using proxy: HTTP=$httpProxy  HTTPS=$httpsProxy" -ForegroundColor Green

# --- Current session env vars ---
$env:HTTP_PROXY  = $httpProxy;  $env:http_proxy  = $httpProxy
$env:HTTPS_PROXY = $httpsProxy; $env:https_proxy = $httpsProxy
$env:ALL_PROXY   = $httpProxy;  $env:all_proxy   = $httpProxy
$env:NO_PROXY    = $noProxy;    $env:no_proxy    = $noProxy

# --- Persist (optional) ---
if ($Persist) {
    Write-Host '[*] Writing user-level persistent env vars...' -ForegroundColor Cyan
    [Environment]::SetEnvironmentVariable('HTTP_PROXY',  $httpProxy,  'User')
    [Environment]::SetEnvironmentVariable('HTTPS_PROXY', $httpsProxy, 'User')
    [Environment]::SetEnvironmentVariable('ALL_PROXY',   $httpProxy,  'User')
    [Environment]::SetEnvironmentVariable('NO_PROXY',    $noProxy,    'User')
}

# --- git ---
if (Get-Command git -ErrorAction SilentlyContinue) {
    git config --global http.proxy  $httpProxy
    git config --global https.proxy $httpsProxy
    Write-Host '[+] git proxy configured.' -ForegroundColor Green
}

# --- npm ---
if (Get-Command npm -ErrorAction SilentlyContinue) {
    npm config set proxy       $httpProxy
    npm config set https-proxy $httpsProxy
    Write-Host '[+] npm proxy configured.' -ForegroundColor Green
}

# --- pip ---
if (Get-Command pip -ErrorAction SilentlyContinue) {
    pip config set global.proxy $httpProxy 2>$null
    Write-Host '[+] pip proxy configured.' -ForegroundColor Green
}

Write-Host ''
Write-Host '[OK] Done. Current session is active; with -Persist new terminals will inherit it too.' -ForegroundColor Green
