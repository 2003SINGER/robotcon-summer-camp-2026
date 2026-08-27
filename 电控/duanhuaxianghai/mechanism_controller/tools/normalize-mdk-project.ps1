param(
  [string]$ProjectFile = (Join-Path (Split-Path -Parent $PSScriptRoot) 'MDK-ARM\mechanism_controller.uvprojx')
)

$ErrorActionPreference = 'Stop'
[xml]$project = Get-Content -LiteralPath $ProjectFile
$groups = $project.SelectSingleNode('//Groups')
if ($null -eq $groups) { throw "No <Groups> node in $ProjectFile" }

function Get-Group([string]$name) {
  return @($groups.SelectNodes('./Group') | Where-Object {
    $_.SelectSingleNode('./GroupName').InnerText -eq $name
  })
}

# CubeMX owns these groups.  When a project has been generated with several
# toolchain selections, old hand-maintained Core/Drivers groups coexist with
# CubeMX's Application/User and Drivers groups, compiling every HAL file twice.
$legacyCore = @(Get-Group 'Core/Src') | Select-Object -First 1
$legacyInc = @(Get-Group 'Core/Inc') | Select-Object -First 1
if ($legacyCore.Count -ne 1 -or $legacyInc.Count -ne 1) {
  throw 'Expected legacy Core/Src and Core/Inc groups were not found; refusing to alter the Keil project.'
}

$appSrc = $legacyCore.CloneNode($true)
$appSrc.SelectSingleNode('./GroupName').InnerText = 'App/Src'
@($appSrc.SelectNodes('./Files/File')) | Where-Object {
  $_.SelectSingleNode('./FilePath').InnerText -notmatch '(^|[\\/])App[\\/]Src[\\/]'
} | ForEach-Object { [void]$appSrc.SelectSingleNode('./Files').RemoveChild($_) }

$appInc = $legacyInc.CloneNode($true)
$appInc.SelectSingleNode('./GroupName').InnerText = 'App/Inc'
@($appInc.SelectNodes('./Files/File')) | Where-Object {
  $_.SelectSingleNode('./FilePath').InnerText -notmatch '(^|[\\/])App[\\/]Inc[\\/]'
} | ForEach-Object { [void]$appInc.SelectSingleNode('./Files').RemoveChild($_) }

if ($appSrc.SelectNodes('./Files/File').Count -eq 0 -or $appInc.SelectNodes('./Files/File').Count -eq 0) {
  throw 'App source/header extraction produced an empty group; refusing to alter the Keil project.'
}

@('Core/Src', 'Drivers/STM32H7xx_HAL_Driver/Src', 'Core/Inc', 'App/Src', 'App/Inc') | ForEach-Object {
  @(Get-Group $_) | ForEach-Object { [void]$groups.RemoveChild($_) }
}
[void]$groups.AppendChild($appSrc)
[void]$groups.AppendChild($appInc)

$writerSettings = [System.Xml.XmlWriterSettings]::new()
$writerSettings.Encoding = [System.Text.UTF8Encoding]::new($false)
$writerSettings.Indent = $true
$writer = [System.Xml.XmlWriter]::Create($ProjectFile, $writerSettings)
try { $project.Save($writer) } finally { $writer.Dispose() }

[xml]$verified = Get-Content -LiteralPath $ProjectFile
$paths = @($verified.SelectNodes('//File/FilePath') | ForEach-Object { $_.InnerText })
$duplicates = @($paths | Group-Object | Where-Object Count -gt 1)
if ($duplicates.Count -ne 0) { throw "Normalization failed: $($duplicates.Count) duplicate source path(s) remain." }
"Normalized ${ProjectFile}: $($paths.Count) unique project files."
