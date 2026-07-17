# Install Hyper-V PowerShell module
$output = @()
$output += "========================================="
$output += "Installing Hyper-V PowerShell Module"
$output += "========================================="

# Check if we can use DISM
$output += ""
$output += "Checking DISM..."
$dismPath = "C:\Windows\System32\dism.exe"
if (Test-Path $dismPath) {
    $output += "DISM found: $dismPath"
    
    # Try to enable Hyper-V management tools
    $output += ""
    $output += "Enabling Hyper-V Management Tools..."
    $result = & $dismPath /Online /Enable-Feature /FeatureName:Microsoft-Hyper-V-Tools-All /NoRestart 2>&1
    $output += $result
    
    $output += ""
    $output += "Enabling Hyper-V PowerShell module..."
    $result2 = & $dismPath /Online /Enable-Feature /FeatureName:Microsoft-Hyper-V-PowerShell /NoRestart 2>&1
    $output += $result2
} else {
    $output += "DISM not found"
}

$output += ""
$output += "Done!"
$output | Out-File "D:\oezcon\hyperv_install.txt"
