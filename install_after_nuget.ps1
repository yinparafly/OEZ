# Check NuGet and install Hyper-V module
$output = @()

# Check NuGet
$output += "Checking NuGet..."
try {
    $nuget = Get-PackageProvider -Name NuGet -ErrorAction Stop
    $output += "NuGet installed: $($nuget.Version)"
} catch {
    $output += "NuGet not found, installing..."
    try {
        Install-PackageProvider -Name NuGet -MinimumVersion 2.8.5.201 -Force -Scope CurrentUser -ErrorAction Stop
        $output += "NuGet installed!"
    } catch {
        $output += "Failed to install NuGet: $_"
    }
}

# Install Hyper-V module
$output += ""
$output += "Installing Hyper-V module..."
try {
    Install-Module -Name Hyper-V -Force -AllowClobber -Scope CurrentUser -ErrorAction Stop
    $output += "Hyper-V module installed!"
    
    # Test it
    Import-Module Hyper-V -ErrorAction Stop
    if (Get-Command Get-VM -ErrorAction SilentlyContinue) {
        $output += "Module works! Get-VM available."
        Get-VM | ForEach-Object { $output += "  VM: $($_.Name) - $($_.State)" }
    }
} catch {
    $output += "Failed to install Hyper-V module: $_"
}

$output | Out-File "D:\oezcon\nuget_result.txt"
