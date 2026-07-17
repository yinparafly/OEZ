# Enable Hyper-V with output to file
$output = @()
$output += "========================================="
$output += "Enabling Hyper-V"
$output += "========================================="

try {
    $feature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All -ErrorAction Stop
    $output += "Current state: $($feature.State)"
    
    if ($feature.State -eq "Enabled") {
        $output += "Hyper-V already enabled!"
    } else {
        $output += "Enabling Hyper-V..."
        $result = Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All -All -NoRestart -ErrorAction Stop
        $output += "Result: $($result.RestartNeeded)"
        $output += "Hyper-V enabled!"
        $output += ""
        $output += "You may need to restart your computer."
        $output += "After restart, run install_win11.bat again."
    }
} catch {
    $output += "Error: $_"
}

$output += ""
$output += "Done!"
$output | Out-File "D:\oezcon\hyperv_enable.txt"
