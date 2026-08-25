# Windows twin of journal.sh: the ChaCha20 core against the RFC 8439
# vector (built with the WSL distro's gcc), the CMOC --os9 module build
# and disk layout with the toolshed os9 tool in the same distro (see
# tools/ecr-to-wsl.ps1), then the create/seek/write syscall probe and
# the full journal write / append / read / wrong-passphrase runs in
# headless NitrOS-9, finishing with a plaintext sweep of the raw disk
# image. The journal disk rides in /d1 via the VCC_DISK1 override.
#
# Usage: tests/journal.ps1 [-Frames 15000] [-Rom C:\roms\coco3.rom]
#        [-BootDisk <NOS9 emudsk boot dsk>] [-Vhd <NitrOS9.vhd>]
#        Env (VCC_NO_JIT=1, VCC_NO_BURST=1, ...) passes through.

param(
    [int]$Frames = 15000,
    [string]$Rom = "C:\roms\coco3.rom",
    [string]$BootDisk = "C:\vcc_deployed\VHD-Emudisk Install Disks\NOS9_6309_L2_v030300_coco3_emudsk_boot.dsk",
    [string]$Vhd = "C:\vcc_deployed\disks\NitrOS9.vhd",
    [string]$CmocDistro = "cmoc",
    [string]$BuildDir = ""
)

$ErrorActionPreference = "Stop"
$tests = $PSScriptRoot
$repo = (Resolve-Path (Join-Path $tests "..")).Path
if (-not $BuildDir) { $BuildDir = Join-Path $repo "build-x64\Release" }
$exe = Join-Path $BuildDir "vcc-headless.exe"
if (-not (Test-Path $exe)) { throw "vcc-headless not found at $exe" }

$W = Join-Path $env:TEMP "vcc-journal-test"
New-Item -ItemType Directory -Force $W | Out-Null
$wsl = "/mnt/" + ($W.Substring(0,1).ToLower()) + ($W.Substring(2) -replace "\\","/")

function Ok([string]$m)  { Write-Output "  ok: $m" }
function Bad([string]$m) { Write-Output "  FAIL: $m"; exit 1 }

# 1. ChaCha20 against the RFC 8439 section 2.3.2 vector. The distro's
# gcc stands in for the host cc; the vector math is host-agnostic.
# chacha_host.c includes ../apps/journal/chacha.h, so compile from a
# tests/ dir in a replica of the repo layout.
New-Item -ItemType Directory -Force (Join-Path $W "tests"), (Join-Path $W "apps\journal") | Out-Null
Copy-Item (Join-Path $repo "tests\chacha_host.c") (Join-Path $W "tests") -Force
Copy-Item (Join-Path $repo "apps\journal\chacha.c") (Join-Path $W "apps\journal") -Force
Copy-Item (Join-Path $repo "apps\journal\chacha.h") (Join-Path $W "apps\journal") -Force
$r = wsl -d $CmocDistro -- /bin/sh -c "cd '$wsl/tests' && gcc -O2 -o chacha_host chacha_host.c ../apps/journal/chacha.c && ./chacha_host"
if ($LASTEXITCODE -ne 0 -or ($r -notmatch "PASS")) { Bad "chacha RFC vector" }
Ok "chacha RFC vector"

# 2. Build the modules and a fresh OS-9 disk with the WSL cmoc tools.
foreach ($f in "journal.c","chacha.c","chacha.h","os9sys.c","os9sys.h","seektest.c") {
    Copy-Item (Join-Path $repo "apps\journal\$f") $W -Force
}
wsl -d $CmocDistro -- /bin/sh -c "cd '$wsl' && cmoc --os9 -o seektest seektest.c os9sys.c && cmoc --os9 -o journal journal.c chacha.c os9sys.c && rm -f j.dsk && os9 format -e -t35 -ss -dd -q j.dsk && os9 makdir j.dsk,CMDS && os9 copy journal j.dsk,CMDS/journal && os9 copy seektest j.dsk,CMDS/seektest && os9 attr -q -e -pe -r -pr j.dsk,CMDS/journal && os9 attr -q -e -pe -r -pr j.dsk,CMDS/seektest" | Out-Null
if ($LASTEXITCODE -ne 0) { Bad "wsl cmoc/os9 build" }
Ok "wsl cmoc/os9 build"

# A NitrOS-9-booting machine: 6309, MPI with the emudsk VHD in slot 3
# and the FD-502 (boot floppy in drive 0) in slot 4. RTC off so runs
# stay bit-identical. Forward slashes for the ini reader.
$bootIni = $BootDisk -replace "\\","/"
$vhdIni  = $Vhd -replace "\\","/"
$romIni  = $Rom -replace "\\","/"
$slotDir = $BuildDir -replace "\\","/"
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
SLOT3=$slotDir/harddisk.dll
SLOT4=$slotDir/fd502.dll
[Hard Drive]
VHDImage=$vhdIni
ClkEnable=0
ClkRdOnly=1
[FD-502]
DiskRom=1
Persist=1
Disk#0=$bootIni
ClkEnable=0
TurboDisk=1
[DW Becker]
DWEnable=0
"@
[IO.File]::WriteAllText((Join-Path $W "journal.ini"), $ini)

# Boot NitrOS-9 from the floppy, wait out the boot, land in /d1/cmds.
$BOOT = "DOS`n" + ("~" * 46) + "chx /d1/cmds`n~~"

function Run-OS9([string]$keys, [string]$shot) {
    $env:VCC_INI = Join-Path $W "journal.ini"
    $env:VCC_DISK1 = Join-Path $W "j.dsk"
    if (-not $env:VCC_FRAMESKIP) { $env:VCC_FRAMESKIP = "100" }
    $env:VCC_SHOT_TEXT = Join-Path $W $shot
    # Native stderr + $ErrorActionPreference=Stop is a terminating
    # error in PowerShell 5.1; relax around the call and fold streams.
    $eap = $ErrorActionPreference; $ErrorActionPreference = "Continue"
    & $exe $Rom $Frames ($BOOT + $keys) 2>&1 | Out-Null
    $ErrorActionPreference = $eap
    $env:VCC_SHOT_TEXT = $null
    if (-not (Test-Path (Join-Path $W $shot))) { Bad "no screen dump ($shot)" }
}

# 3. Syscall probe: create, size, seek, write, size on a real file.
Run-OS9 "seektest`n~~~~~~~~" "seek.txt"
if (-not (Select-String -Path "$W\seek.txt" -Pattern "SIZE2 16" -Quiet)) { Bad "seektest (create/seek/write)" }
Ok "seektest (create/seek/write)"

# 4. Write an entry, append a second, read both, reject a bad pass.
# The scripted CoCo keyboard delivers lowercase, so the read-back
# matches are lowercase too.
Run-OS9 "journal -d /d1`n~~~~swordfish`n~~~~FIRST SECRET ENTRY.`n~~.`n~~~~~~~~~~" "w.txt"
if (-not (Select-String -Path "$W\w.txt" -Pattern "Saved \(" -Quiet)) { Bad "journal write" }
Ok "journal write"

Run-OS9 "journal -d /d1`n~~~~swordfish`n~~~~SECOND SECRET ENTRY.`n~~.`n~~~~~~~~~~" "a.txt"
if (-not (Select-String -Path "$W\a.txt" -Pattern "Saved \(" -Quiet)) { Bad "journal append" }
Ok "journal append"

Run-OS9 "journal -d /d1 read`n~~~~swordfish`n~~~~~~~~~~" "r.txt"
if (-not (Select-String -Path "$W\r.txt" -Pattern "first secret entry" -Quiet)) { Bad "read: first entry" }
if (-not (Select-String -Path "$W\r.txt" -Pattern "second secret entry" -Quiet)) { Bad "read: second entry" }
Ok "journal read (both entries)"

Run-OS9 "journal -d /d1 read`n~~~~wrongpass`n~~~~~~~~~~" "x.txt"
if (-not (Select-String -Path "$W\x.txt" -Pattern "\(wrong passphrase\)" -Quiet)) { Bad "wrong passphrase rejected" }
if (Select-String -Path "$W\x.txt" -Pattern "secret entry" -Quiet) { Bad "wrong passphrase leaked plaintext" }
Ok "wrong passphrase rejected"

# 5. The raw disk image must hold ciphertext only. Case-insensitive
# byte sweep of the image, the strings(1) way.
$bytes = [IO.File]::ReadAllBytes((Join-Path $W "j.dsk"))
$ascii = [Text.Encoding]::ASCII.GetString($bytes)
if ($ascii -match "(?i)secret entry") { Bad "plaintext found on disk image" }
Ok "ciphertext only on disk"

Write-Output "journal: PASS"
