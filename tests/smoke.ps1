# Windows twin of smoke.sh: boots the portable core through three
# deepening scenarios and greps the dumped text screens for expected
# content. Timestamps and disk activity vary run to run, so the
# assertions are presence-based. Generates its own machine ini (MPI +
# FD502 boot floppy + VHD) instead of relying on a default vcc.ini.
#
# Usage: tests/smoke.ps1 [-Rom C:\roms\coco3.rom]
#        [-BootDisk <NOS9 emudsk boot dsk>] [-Vhd <NitrOS9.vhd>]

param(
    [string]$Rom = "C:\roms\coco3.rom",
    [string]$BootDisk = "C:\vcc_deployed\VHD-Emudisk Install Disks\NOS9_6309_L2_v030300_coco3_emudsk_boot.dsk",
    [string]$Vhd = "C:\vcc_deployed\disks\NitrOS9.vhd",
    [string]$BuildDir = ""
)

$ErrorActionPreference = "Continue"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if (-not $BuildDir) { $BuildDir = Join-Path $repo "build-x64\Release" }
$exe = Join-Path $BuildDir "vcc-headless.exe"
if (-not (Test-Path $exe)) { Write-Output "vcc-headless not found at $exe"; exit 1 }

$W = Join-Path $env:TEMP "vcc-smoke-test"
New-Item -ItemType Directory -Force $W | Out-Null

$romIni = $Rom -replace "\\","/"
$ini = @"
[CPU]
CpuType=1
[Memory]
RamSize=2
[Misc]
ExternalBasicImage=$romIni
AutoStart=1
CartAutoStart=0
[Module]
OnBoot=mpi.dll
[MPI]
SWPOSITION=3
PesistPaks=1
SLOT3=$($BuildDir -replace "\\","/")/harddisk.dll
SLOT4=$($BuildDir -replace "\\","/")/fd502.dll
[Hard Drive]
VHDImage=$($Vhd -replace "\\","/")
ClkEnable=0
ClkRdOnly=1
[FD-502]
DiskRom=1
Persist=1
Disk#0=$($BootDisk -replace "\\","/")
ClkEnable=0
TurboDisk=1
[DW Becker]
DWEnable=0
"@
[IO.File]::WriteAllText((Join-Path $W "smoke.ini"), $ini)
$env:VCC_INI = Join-Path $W "smoke.ini"
if (-not $env:VCC_FRAMESKIP) { $env:VCC_FRAMESKIP = "100" }

$script:Fails = 0
function Run-Expect([string]$name, [int]$frames, [string]$keys, [string[]]$expects) {
    $out = (& $exe $Rom $frames $keys 2>&1 | ForEach-Object { "$_" }) -join "`n"
    foreach ($e in $expects) {
        if ($out.Contains($e)) {
            Write-Output "PASS  $name`: $e"
        } else {
            Write-Output "FAIL  $name`: missing `"$e`""
            ($out -split "`n") | Select-Object -Last 30 | Write-Output
            $script:Fails++
        }
    }
}

# 1. Disk BASIC banner (core + MPI + FD502 ROM mapping).
Run-Expect "basic" 600 "" @(
    "DISK EXTENDED COLOR BASIC 2.1",
    "COPR. 1982, 1986 BY TANDY")

# 2. NitrOS-9 Level 2 boot from the FD502 floppy, /DD on the VHD,
#    module directory via mdir (kernel, disk drivers, windows, clock).
Run-Expect "nitros9" 4200 "DOS`n~~~~~~~~~~mdir`n~~" @(
    "NitrOS-9/6309 Level 2",
    "Shell+",
    "EmuDsk",
    "KrnP2")

# 3. Basic09 from /DD/CMDS.
Run-Expect "basic09" 4800 "DOS`n~~~~~~~~~~basic09`n~~~~" @(
    "BASIC09",
    "COPYRIGHT 1980 BY MOTOROLA INC.",
    "Ready")

if ($script:Fails -eq 0) {
    Write-Output "smoke: all scenarios passed"
} else {
    Write-Output "smoke: $($script:Fails) assertion(s) FAILED"
    exit 1
}
