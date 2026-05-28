$ErrorActionPreference = 'Stop'

$root = $PSScriptRoot
$projectPath = Join-Path $root 'VMT_SETO\VMT_SETO.csproj'
$project = [xml](Get-Content -LiteralPath $projectPath)
$version = [string]$project.Project.PropertyGroup.Version
if ([string]::IsNullOrWhiteSpace($version)) {
    $version = '1.0.0'
}

$packageName = "VMT_SETO_v$version"
$distRoot = Join-Path $root 'dist'
$publishDir = Join-Path $distRoot $packageName
$zipPath = Join-Path $distRoot "$packageName`_Booth.zip"

New-Item -ItemType Directory -Force -Path $distRoot | Out-Null

if (Test-Path -LiteralPath $publishDir) {
    $resolvedPublish = (Resolve-Path -LiteralPath $publishDir).Path
    $resolvedDist = (Resolve-Path -LiteralPath $distRoot).Path
    if (-not $resolvedPublish.StartsWith($resolvedDist, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove path outside dist: $resolvedPublish"
    }

    Remove-Item -LiteralPath $resolvedPublish -Recurse -Force
}

dotnet publish $projectPath -c Release -r win-x64 --self-contained true -o $publishDir -p:PublishSingleFile=true -p:IncludeNativeLibrariesForSelfExtract=true -p:EnableCompressionInSingleFile=true -p:DebugType=None -p:DebugSymbols=false -clp:ErrorsOnly

Copy-Item -LiteralPath (Join-Path $root 'README.md') -Destination $publishDir -Force
Copy-Item -LiteralPath (Join-Path $root 'LICENSE') -Destination $publishDir -Force
Copy-Item -LiteralPath (Join-Path $root 'THIRD_PARTY_NOTICES.md') -Destination $publishDir -Force

Get-ChildItem -LiteralPath $publishDir -Filter '*.pdb' -File | Remove-Item -Force

if (Test-Path -LiteralPath $zipPath) {
    Remove-Item -LiteralPath $zipPath -Force
}

Compress-Archive -Path (Join-Path $publishDir '*') -DestinationPath $zipPath -Force

Write-Host "Package: $zipPath"
