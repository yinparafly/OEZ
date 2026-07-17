# List all Hyper-V related features
$output = @()
$output += "Listing all Hyper-V related Windows features..."
$output += ""

# Get all features
$features = Get-WindowsOptionalFeature -Online | Where-Object { $_.FeatureName -like '*Hyper*' }
if ($features) {
    $output += "Hyper-V features found:"
    $features | ForEach-Object {
        $output += "  $($_.FeatureName) - State: $($_.State)"
    }
} else {
    $output += "No Hyper-V features found in optional features"
}

# Also check installed features
$output += ""
$output += "Checking installed Windows features..."
$installed = Get-WindowsFeature | Where-Object { $_.Name -like '*Hyper*' }
if ($installed) {
    $installed | ForEach-Object {
        $output += "  $($_.Name) - Installed: $($_.Installed)"
    }
} else {
    $output += "No Hyper-V features found in Windows features"
}

# Check for PowerShell module in common locations
$output += ""
$output += "Searching for Hyper-V PowerShell module..."
$locations = @(
    "C:\Windows\System32\WindowsPowerShell\v1.0\Modules\Hyper-V",
    "C:\Program Files\WindowsPowerShell\Modules\Hyper-V",
    "C:\Users\$env:USERNAME\Documents\WindowsPowerShell\Modules\Hyper-V"
)
foreach ($loc in $locations) {
    if (Test-Path $loc) {
        $output += "  Found: $loc"
    }
}

$output | Out-File "D:\oezcon\hyperv_list.txt"
