# Admin check with file output
$output = @()

$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
$output += "Admin: $isAdmin"

if ($isAdmin) {
    try {
        $feature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All -ErrorAction Stop
        $output += "Hyper-V All: $($feature.State)"
    } catch {
        $output += "Hyper-V All check failed: $_"
    }
    
    $modulePath = "C:\Windows\System32\WindowsPowerShell\v1.0\Modules\Hyper-V"
    $output += "Module path exists: $(Test-Path $modulePath)"
    
    if (Test-Path $modulePath) {
        Import-Module "$modulePath\Hyper-V.psd1" -ErrorAction SilentlyContinue
        if (Get-Command Get-VM -ErrorAction SilentlyContinue) {
            $output += "Hyper-V module loaded!"
            Get-VM | ForEach-Object { $output += "VM: $($_.Name) - $($_.State)" }
        } else {
            $output += "Failed to load Hyper-V module"
        }
    }
} else {
    $output += "Need admin"
}

$output | Out-File "D:\oezcon\hyperv_check.txt"
