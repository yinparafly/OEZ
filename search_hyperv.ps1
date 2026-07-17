# Find actual Hyper-V module files
$output = @()
$output += "Searching for Hyper-V PowerShell module files..."

# Search in common locations
$searchPaths = @(
    "C:\Windows\System32\WindowsPowerShell",
    "C:\Windows\SysWOW64\WindowsPowerShell",
    "C:\Program Files",
    "C:\Program Files (x86)"
)

foreach ($path in $searchPaths) {
    if (Test-Path $path) {
        $found = Get-ChildItem -Path $path -Recurse -Filter "Hyper-V.psd1" -ErrorAction SilentlyContinue | Select-Object -First 5
        if ($found) {
            $output += "Found in ${path}:"
            $found | ForEach-Object { $output += "  $($_.FullName)" }
        }
    }
}

# Also check if we can use DISM to get more info
$output += ""
$output += "Checking DISM for Hyper-V package info..."
$dismOutput = & dism /online /get-features /format:table 2>&1 | Select-String -Pattern "Hyper"
if ($dismOutput) {
    $dismOutput | ForEach-Object { $output += "  $_" }
}

# Try to manually import from System32
$output += ""
$output += "Trying to import from System32..."
$sysPath = "C:\Windows\System32\WindowsPowerShell\v1.0\Modules"
if (Test-Path $sysPath) {
    $modules = Get-ChildItem $sysPath | Where-Object { $_.Name -like "*Hyper*" }
    if ($modules) {
        $modules | ForEach-Object { $output += "  $($_.Name)" }
    } else {
        $output += "  No Hyper-V modules in System32\Modules"
    }
}

# Check Windows Module Service
$output += ""
$output += "Checking Windows Module Installer service..."
$service = Get-Service -Name "TrustedInstaller" -ErrorAction SilentlyContinue
if ($service) {
    $output += "  TrustedInstaller: $($service.Status)"
}

$output | Out-File "D:\oezcon\hyperv_search.txt"
