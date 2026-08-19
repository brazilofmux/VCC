# Pull a container image from ECR and import it as a WSL1 distro -
# no docker/podman needed, which matters on EC2 instances without
# nested virtualization (WSL2/containers won't run on e.g. t3.*, but
# WSL1 executes Linux userspace binaries fine).
#
# Usage: tools/ecr-to-wsl.ps1 [-Repo cmoc] [-Tag amd64] [-Distro cmoc]
#        [-HelperDistro alpine-test]
#
# Requires: AWS CLI with ECR read access, an existing WSL1 helper
# distro with busybox/tar (any Alpine minirootfs import works) used to
# flatten layers with Linux permissions/symlinks intact - extracting
# on NTFS directly would lose both.
#
# After import: wsl -d cmoc -- cmoc --version

param(
    [string]$Repo = "cmoc",
    [string]$Tag = "amd64",
    [string]$Distro = "cmoc",
    [string]$HelperDistro = "alpine-test",
    [string]$Arch = "amd64",
    [string]$WslRoot = "C:\wsl"
)

$ErrorActionPreference = "Stop"
# PS 5.1's download progress rendering slows Invoke-WebRequest by an
# order of magnitude on large blobs.
$ProgressPreference = "SilentlyContinue"
$work = Join-Path $env:TEMP "ecr-pull-$Repo-$Tag"
New-Item -ItemType Directory -Force $work | Out-Null

Write-Host "fetching manifest for ${Repo}:${Tag}..."
$manifestJson = aws ecr batch-get-image --repository-name $Repo `
    --image-ids imageTag=$Tag --query "images[0].imageManifest" --output text
if (-not $manifestJson -or $manifestJson -eq "None") {
    throw "no image ${Repo}:${Tag} in ECR"
}
$manifest = $manifestJson | ConvertFrom-Json

# Multi-arch index: descend to the matching architecture's manifest.
if ($manifest.manifests) {
    $entry = $manifest.manifests | Where-Object { $_.platform.architecture -eq $Arch } | Select-Object -First 1
    if (-not $entry) { throw "index has no $Arch manifest" }
    Write-Host "index -> $Arch manifest $($entry.digest)"
    $manifestJson = aws ecr batch-get-image --repository-name $Repo `
        --image-ids imageDigest=$($entry.digest) --query "images[0].imageManifest" --output text
    $manifest = $manifestJson | ConvertFrom-Json
}

# Sanity: the config blob names the architecture; refuse a mismatch
# (a WSL1 import of an arm64 rootfs fails only at first exec).
$cfgUrl = aws ecr get-download-url-for-layer --repository-name $Repo `
    --layer-digest $manifest.config.digest --query downloadUrl --output text
$cfg = Invoke-RestMethod -Uri $cfgUrl
if ($cfg.architecture -and $cfg.architecture -ne $Arch) {
    throw "image is $($cfg.architecture), wanted $Arch"
}

$layers = @($manifest.layers.digest)
Write-Host "downloading $($layers.Count) layer(s)..."
$i = 0
$layerFiles = @()
foreach ($d in $layers) {
    $f = Join-Path $work ("layer{0:d2}.tar.gz" -f $i)
    if (-not (Test-Path $f)) {
        $url = aws ecr get-download-url-for-layer --repository-name $Repo `
            --layer-digest $d --query downloadUrl --output text
        Invoke-WebRequest -Uri $url -OutFile $f -UseBasicParsing
    }
    $layerFiles += $f
    $i++
}

# Flatten inside the helper distro so Linux metadata survives, then
# tar the result back out for wsl --import. OCI whiteouts (.wh.*)
# delete the shadowed path from lower layers.
Write-Host "flattening layers in $HelperDistro..."
$workWsl = "/mnt/" + ($work.Substring(0,1).ToLower()) + ($work.Substring(2) -replace "\\","/")
$script = @'
set -e
rm -rf /tmp/rootfs && mkdir -p /tmp/rootfs
for f in WORK/layer*.tar.gz; do
    tar -xzf "$f" -C /tmp/rootfs
    find /tmp/rootfs -name ".wh.*" | while read -r wh; do
        target="$(dirname "$wh")/$(basename "$wh" | sed s/^\.wh\.//)"
        rm -rf "$target" "$wh"
    done
done
tar -C /tmp/rootfs -cf WORK/rootfs.tar .
rm -rf /tmp/rootfs
'@ -replace "WORK", $workWsl
$script = $script -replace "`r", ""
# Write the script as UTF-8 WITHOUT a BOM - piping a PowerShell string
# into wsl stdin prepends one, and /bin/sh reads "<BOM>set -e" as an
# unknown command, silently losing the errexit.
$flattenPath = Join-Path $work "flatten.sh"
[IO.File]::WriteAllText($flattenPath, $script)
wsl -d $HelperDistro -- /bin/sh "$workWsl/flatten.sh"
if ($LASTEXITCODE -ne 0) { throw "layer flattening failed" }

Write-Host "importing as WSL1 distro '$Distro'..."
# wsl.exe emits UTF-16; strip the interleaved NULs before comparing.
$existing = (wsl -l -q | Out-String) -replace "`0", "" -split "`r?`n" | Where-Object { $_ -eq $Distro }
if ($existing) { wsl --unregister $Distro | Out-Null }
New-Item -ItemType Directory -Force (Join-Path $WslRoot $Distro) | Out-Null
wsl --import $Distro (Join-Path $WslRoot $Distro) (Join-Path $work "rootfs.tar") --version 1
if ($LASTEXITCODE -ne 0) { throw "wsl --import failed" }

Write-Host "done. try: wsl -d $Distro -- cmoc --version"
