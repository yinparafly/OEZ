# Find Hyper-V module
$output = @()

# Check all module paths
$output += "PowerShell Module Paths:"
$env:PSModulePath -split ';' | ForEach-Object { $output += "  $_" }

# Search for Hyper-V module
$output += ""
$output += "Searching for Hyper-V module..."
Get-Module -ListAvailable | Where-Object { $_.Name -like '*Hyper*' } | ForEach-Object {
    $output += "  Found: $($_.Name) at $($_.Path)"
}

# Check system directory
$sysDir = "C:\Windows\System32\WindowsPowerShell\v1.0\Modules"
$output += ""
$output += "System modules directory: $sysDir"
if (Test-Path $sysDir) {
    Get-ChildItem $sysDir | Where-Object { $_.Name -like '*Hyper*' } | ForEach-Object {
        $output += "  $($_.Name)"
    }
} else {
    $output += "  Directory not found"
}

# Try to find in Windows module path
$winModulePath = "C:\Windows\System32\WindowsPowerShell\Modules"
$output += ""
$output += "Windows PowerShell Modules: $winModulePath"
if (Test-Path $winModulePath) {
    Get-ChildItem $winModulePath | Where-Object { $_.Name -like '*Hyper*' } | ForEach-Object {
        $output += "  $($_.Name)"
    }
} else {
    $output += "  Directory not found"
}

$output | Out-File "D:\oezcon\hyperv_module.txt"
