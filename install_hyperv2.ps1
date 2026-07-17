# Try to install Hyper-V module via PowerShell Gallery or other methods
$output = @()
$output += "Attempting to install Hyper-V PowerShell module..."

# Method 1: Try PowerShellGet
$output += ""
$output += "Method 1: PowerShellGet"
try {
    Install-Module -Name Hyper-V -Force -AllowClobber -Scope CurrentUser -ErrorAction Stop
    $output += "Installed via PowerShellGet!"
} catch {
    $output += "PowerShellGet failed: $($_.Exception.Message)"
}

# Method 2: Try to find and register the module manually
$output += ""
$output += "Method 2: Manual registration"
$moduleDir = "C:\Windows\System32\WindowsPowerShell\v1.0\Modules\Hyper-V"
if (-not (Test-Path $moduleDir)) {
    # Try to create the directory and copy files
    $sourcePaths = @(
        "C:\Windows\System32\WindowsPowerShell\Modules\Hyper-V",
        "C:\Program Files\WindowsPowerShell\Modules\Hyper-V"
    )
    foreach ($src in $sourcePaths) {
        if (Test-Path $src) {
            Copy-Item -Path $src -Destination $moduleDir -Recurse -Force
            $output += "Copied module from $src"
            break
        }
    }
}

# Method 3: Use Windows Package Manager
$output += ""
$output += "Method 3: Checking winget..."
$wingetPath = Get-Command winget -ErrorAction SilentlyContinue
if ($wingetPath) {
    $output += "winget found at $($wingetPath.Source)"
    $output += "Note: winget may not have Hyper-V module"
} else {
    $output += "winget not found"
}

# Method 4: Direct file copy from Windows installation media
$output += ""
$output += "Method 4: Checking for Windows installation files..."
$winSxS = "C:\Windows\WinSxS"
if (Test-Path $winSxS) {
    $hypervFiles = Get-ChildItem $winSxS -Recurse -Filter "*Hyper-V*" -ErrorAction SilentlyContinue | Select-Object -First 5
    if ($hypervFiles) {
        $output += "Found Hyper-V files in WinSxS:"
        $hypervFiles | ForEach-Object { $output += "  $($_.FullName)" }
    } else {
        $output += "No Hyper-V files in WinSxS"
    }
}

$output | Out-File "D:\oezcon\hyperv_install2.txt"
