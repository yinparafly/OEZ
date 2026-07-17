# Manually copy Hyper-V module from Windows installation
$output = @()
$output += "Searching for Hyper-V module files in Windows installation..."

# Check WinSxS directory
$winSxS = "C:\Windows\WinSxS"
$hypervFiles = Get-ChildItem $winSxS -Recurse -Filter "Hyper-V.psd1" -ErrorAction SilentlyContinue | Select-Object -First 5
if ($hypervFiles) {
    $output += "Found Hyper-V module in WinSxS:"
    $hypervFiles | ForEach-Object { 
        $output += "  $($_.FullName)"
        $dir = $_.DirectoryName
        $output += "  Directory: $dir"
        
        # Try to copy to module path
        $targetDir = "C:\Program Files\WindowsPowerShell\Modules\Hyper-V"
        if (-not (Test-Path $targetDir)) {
            New-Item -ItemType Directory -Path $targetDir -Force | Out-Null
        }
        
        # Copy all Hyper-V files
        Copy-Item -Path "$dir\*" -Destination $targetDir -Recurse -Force
        $output += "  Copied to: $targetDir"
    }
} else {
    $output += "No Hyper-V module found in WinSxS"
    
    # Try alternative locations
    $altPaths = @(
        "C:\Windows\System32\WindowsPowerShell\Modules\Hyper-V",
        "C:\Program Files\WindowsPowerShell\Modules\Hyper-V"
    )
    foreach ($p in $altPaths) {
        if (Test-Path $p) {
            $output += "Found at: $p"
        }
    }
}

# Test the module
$output += ""
$output += "Testing Hyper-V module..."
try {
    Import-Module Hyper-V -ErrorAction Stop
    if (Get-Command Get-VM -ErrorAction SilentlyContinue) {
        $output += "SUCCESS! Get-VM available."
        Get-VM | ForEach-Object { $output += "  VM: $($_.Name) - $($_.State)" }
    } else {
        $output += "Module loaded but Get-VM not found"
    }
} catch {
    $output += "Failed to load module: $_"
}

$output | Out-File "D:\oezcon\hyperv_manual.txt"
