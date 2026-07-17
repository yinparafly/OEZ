# Simple Hyper-V module check and install
try {
    Import-Module Hyper-V -ErrorAction Stop
    Write-Host "Hyper-V module loaded!"
    Get-VM | ForEach-Object { Write-Host "VM: $($_.Name)" }
} catch {
    Write-Host "Hyper-V module not found. Installing..."
    
    # Try to enable the PowerShell module feature
    try {
        Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-PowerShell -All -NoRestart -ErrorAction Stop
        Write-Host "Hyper-V PowerShell module installed!"
        Import-Module Hyper-V
        Write-Host "Module loaded!"
    } catch {
        Write-Host "Failed to install Hyper-V PowerShell module: $_"
        Write-Host ""
        Write-Host "Please install manually:"
        Write-Host "1. Open Control Panel -> Programs -> Turn Windows features on or off"
        Write-Host "2. Check 'Hyper-V Management Tools'"
        Write-Host "3. Click OK and restart if needed"
    }
}
