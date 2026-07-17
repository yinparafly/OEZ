# Admin check script
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
Write-Host "Admin: $isAdmin"

if ($isAdmin) {
    $feature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All
    Write-Host "Hyper-V All: $($feature.State)"
    
    $feature2 = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V
    Write-Host "Hyper-V: $($feature2.State)"
    
    # Try to load module from system path
    $modulePath = "C:\Windows\System32\WindowsPowerShell\v1.0\Modules\Hyper-V"
    if (Test-Path $modulePath) {
        Write-Host "Module path exists: $modulePath"
        Import-Module "$modulePath\Hyper-V.psd1" -ErrorAction SilentlyContinue
        if (Get-Command Get-VM -ErrorAction SilentlyContinue) {
            Write-Host "Hyper-V module loaded successfully!"
        } else {
            Write-Host "Failed to load Hyper-V module"
        }
    } else {
        Write-Host "Module path not found: $modulePath"
    }
} else {
    Write-Host "Need admin to check Hyper-V"
}
