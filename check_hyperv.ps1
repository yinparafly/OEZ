# Check Hyper-V module availability
$modules = Get-Module -ListAvailable | Where-Object { $_.Name -like '*Hyper*' }
if ($modules) {
    Write-Host "Hyper-V modules found:"
    $modules | ForEach-Object { Write-Host "  $($_.Name) - $($_.Path)" }
} else {
    Write-Host "No Hyper-V modules found"
    Write-Host ""
    Write-Host "Checking if Hyper-V feature is installed..."
    $feature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All -ErrorAction SilentlyContinue
    if ($feature) {
        Write-Host "Feature: $($feature.FeatureName) - State: $($feature.State)"
    } else {
        Write-Host "Cannot check feature status (need admin)"
    }
}
