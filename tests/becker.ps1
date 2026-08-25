# Windows twin of becker.sh: start a loopback listener, boot with
# DWEnable=1, and require the module status line to reach "DW OK".
# Exercises the WINSOCK half of FD502/becker.cpp (the ws2_32 link on
# the CMake x64 build) including the connect-retry path and the
# retry-flag clear on success. The listener is a .NET TcpListener in a
# background job - no Python dependency.
#
# Usage: tests/becker.ps1 [-Frames 90000] [-Rom C:\roms\coco3.rom]

param(
    [int]$Frames = 90000,
    [string]$Rom = "C:\roms\coco3.rom",
    [string]$BuildDir = ""
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
if (-not $BuildDir) { $BuildDir = Join-Path $repo "build-x64\Release" }
$exe = Join-Path $BuildDir "vcc-headless.exe"
if (-not (Test-Path $exe)) { throw "vcc-headless not found at $exe" }

$W = Join-Path $env:TEMP "vcc-becker-test"
New-Item -ItemType Directory -Force $W | Out-Null

$romIni = $Rom -replace "\\","/"
$ini = @"
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
[DW Becker]
DWEnable=1
DWServerAddr=127.0.0.1
DWServerPort=65504
"@
[IO.File]::WriteAllText((Join-Path $W "becker.ini"), $ini)

# Accept-and-drain listener on the becker port, in a job so it
# survives the emulator run and dies with the script.
$serv = Start-Job -ScriptBlock {
    $l = New-Object System.Net.Sockets.TcpListener([System.Net.IPAddress]::Loopback, 65504)
    $l.Start()
    $buf = New-Object byte[] 4096
    while ($true) {
        $c = $l.AcceptTcpClient()
        $s = $c.GetStream()
        try { while ($s.Read($buf, 0, $buf.Length) -gt 0) {} } catch {}
        $c.Close()
    }
}
try {
    Start-Sleep -Seconds 1

    # Enough frames that the emulator runs ~1s of wall time so the
    # connection thread has real time to connect.
    $env:VCC_INI = Join-Path $W "becker.ini"
    if (-not $env:VCC_FRAMESKIP) { $env:VCC_FRAMESKIP = "100" }
    $eap = $ErrorActionPreference; $ErrorActionPreference = "Continue"
    $out = & $exe $Rom $Frames 2>&1 | ForEach-Object { "$_" } |
           Where-Object { $_ -match "module status" }
    $ErrorActionPreference = $eap

    Write-Output $out
    if ($out -match "DW OK") { Write-Output "becker: PASS" }
    else { Write-Output "becker: FAIL"; exit 1 }
}
finally {
    Stop-Job $serv -ErrorAction SilentlyContinue
    Remove-Job $serv -Force -ErrorAction SilentlyContinue
}
