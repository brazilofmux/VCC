# Windows twin of run.sh: build and run the compiled-C CPU benchmark
# (tests/bench/sieve.c) in vcc-headless. Compiles with the cmoc tools
# in a WSL1 distro (see tools/ecr-to-wsl.ps1 for getting one from ECR
# without a container runtime), writes a DECB disk image, points
# vcc-headless at a purpose-built ini via VCC_INI, types LOADM"SIEVE /
# EXEC through the scripted keyboard, and reports the PRIMES result
# plus wall-clock speed.
#
# Usage: tests/bench/run.ps1 [-Frames 12000] [-Rom C:\roms\coco3.rom]
#        Env (VCC_NO_TAKEN=1, VCC_NO_JIT=1, ...) passes through.

param(
    [int]$Frames = 12000,
    [string]$Rom = "C:\roms\coco3.rom",
    [string]$CmocDistro = "cmoc",
    [string]$BuildDir = ""
)

$ErrorActionPreference = "Stop"
$bench = $PSScriptRoot
$repo = (Resolve-Path (Join-Path $bench "..\..")).Path
if (-not $BuildDir) { $BuildDir = Join-Path $repo "build-x64\Release" }
$exe = Join-Path $BuildDir "vcc-headless.exe"
if (-not (Test-Path $exe)) { throw "vcc-headless not found at $exe" }

$work = Join-Path $env:TEMP "vcc-bench"
New-Item -ItemType Directory -Force $work | Out-Null

# Compile + write the disk when the source is newer than the image.
$src = Join-Path $bench "sieve.c"
$dsk = Join-Path $work "sieve.dsk"
if (-not (Test-Path $dsk) -or
    (Get-Item $src).LastWriteTime -gt (Get-Item $dsk).LastWriteTime) {
    Copy-Item $src $work -Force
    # A DECB-formatted blank disk is 35x18x256 bytes of 0xFF (what
    # DSKINI writes): all-0xFF GAT = every granule free, all-0xFF
    # directory = no entries.
    $blank = New-Object byte[] 161280
    for ($i = 0; $i -lt $blank.Length; $i++) { $blank[$i] = 0xFF }
    [IO.File]::WriteAllBytes($dsk, $blank)
    $workWsl = "/mnt/" + ($work.Substring(0,1).ToLower()) + ($work.Substring(2) -replace "\\","/")
    wsl -d $CmocDistro -- /bin/sh -c "cd '$workWsl' && cmoc -o sieve.bin sieve.c && writecocofile sieve.dsk sieve.bin"
    if ($LASTEXITCODE -ne 0) { throw "cmoc build failed" }
}

# Minimal machine config: 6309, 2MB, FD502 alone with the bench disk.
# Forward slashes: the ini reader is happier with them on both hosts.
$dskIni = $dsk -replace "\\", "/"
$romIni = $Rom -replace "\\", "/"
@"
[CPU]
CpuType=1
[Memory]
RamSize=2
[Misc]
ExternalBasicImage=$romIni
CartAutoStart=0
[Module]
OnBoot=fd502.dll
[FD-502]
DiskRom=1
Disk#0=$dskIni
"@ | Set-Content (Join-Path $work "bench.ini") -Encoding ascii

$env:VCC_INI = Join-Path $work "bench.ini"
if (-not $env:VCC_FRAMESKIP) { $env:VCC_FRAMESKIP = "100" }
# Windows PowerShell 5.1 native-arg quoting: an embedded " must be
# passed as \" or it is silently stripped (DECB then sees LOADMSIEVE).
$keys = "LOADM\`"SIEVE`n~~~~~~~~~~EXEC`n"
# PS 5.1 wraps native stderr lines in ErrorRecords, which under
# ErrorActionPreference=Stop turn the shell's harmless boot chatter
# into a terminating error - so relax it and stringify both streams.
$ErrorActionPreference = "Continue"
(& $exe $Rom $Frames $keys 2>&1) | ForEach-Object { "$_" } |
    Select-String "PRIMES|BENCH|realtime|effective|JIT:" | ForEach-Object Line
