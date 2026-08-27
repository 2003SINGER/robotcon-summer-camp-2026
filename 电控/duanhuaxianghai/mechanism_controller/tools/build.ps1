param(
  [ValidateSet('Debug', 'Release')]
  [string]$Configuration = 'Debug'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$cmake = 'E:\STM32CubeCLT_1.21.0\CMake\bin\cmake.exe'
if (-not (Test-Path -LiteralPath $cmake)) { throw "STM32CubeCLT CMake missing: $cmake" }

# CMake is the single GNU source list.  Do not add a second list here: CubeMX
# owns Core/Drivers and the root CMakeLists.txt owns the stable App additions.
Push-Location $projectRoot
try {
  & $cmake '--preset' $Configuration
  if ($LASTEXITCODE -ne 0) { throw "CMake configure exited with $LASTEXITCODE" }
  & $cmake '--build' '--preset' $Configuration
  if ($LASTEXITCODE -ne 0) { throw "CMake build exited with $LASTEXITCODE" }
} finally {
  Pop-Location
}
