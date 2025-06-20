# Accept command-line parameters
param (
    [switch]$DryRun
)

# Build the base command
$command = "compote component upload --name ble_bidi_fast --namespace shoderico"

# Append --dry-run option if specified
if ($DryRun) {
    $command += " --dry-run"
}

# Execute the command
Invoke-Expression $command