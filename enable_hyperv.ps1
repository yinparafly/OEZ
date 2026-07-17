# Enable Hyper-V - Run as Administrator
Write-Host "========================================="
Write-Host "Enabling Hyper-V"
Write-Host "========================================="

# Check current state
$feature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All
Write-Host "Current state: $($feature.State)"

if ($feature.State -eq "Enabled") {
    Write-Host "Hyper-V already enabled!"
} else {
    Write-Host "Enabling Hyper-V (may require restart)..."
    Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All -All -NoRestart
    Write-Host "Hyper-V enabled!"
    Write-Host ""
    Write-Host "IMPORTANT: You may need to restart your computer."
    Write-Host "After restart, run install_win11.bat again."
}

Write-Host ""
Write-Host "Done!"
