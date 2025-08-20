# pointcloud_codec — LiDAR point-cloud quantization + Draco

> Compress `.pcd` point clouds with position quantization, decode back, **visualize before/after**, and measure distortion (**RMSE / Chamfer**). Windows + CMake + vcpkg + PCL/VTK.

---

## TL;DR

- **Encode**: PCD → Draco (`.drc`) with configurable **qpos_bits**
- **Decode**: Draco → PCD
- **View**: Side-by-side viewer (original vs. reconstructed) via **PCL/VTK**
- **Measure**: pointwise **RMSE** and **Chamfer RMSE**
- **Plot**: quick **rate–distortion** CSV sweep across q=8..16

---

## Prereqs

- **Windows 10/11**, **VS 2022**, **CMake ≥ 3.22**
- **vcpkg** installed at `C:\vcpkg` (or set `VCPKG_ROOT` below)

> Tip: Open a **“x64 Native Tools for VS 2022”** or **“VS 2022 Developer PowerShell”**.

---

## Quick start (copy into VS 2022 x64 Developer PowerShell)

> This script configures/builds the tools, downloads a sample PCD, encodes/decodes at `q=14`, opens the viewer, and writes a **rate–distortion** CSV.

```powershell
$ErrorActionPreference = 'Stop'
$repo = (Get-Location).Path
$env:VCPKG_ROOT = $env:VCPKG_ROOT ?? 'C:\vcpkg'
$toolchain = Join-Path $env:VCPKG_ROOT 'scripts\buildsystems\vcpkg.cmake'
if (!(Test-Path $toolchain)) { throw "vcpkg toolchain not found at $toolchain" }

# Minimal package config template required by CMake install() step
New-Item -ItemType Directory -Force -Path .\cmake | Out-Null
$cfgIn = '.\cmake\pointcloud_codecConfig.cmake.in'
if (!(Test-Path $cfgIn)) {
@'
# Minimal package config for pointcloud_codec
include("${CMAKE_CURRENT_LIST_DIR}/pointcloud_codecTargets.cmake")
'@ | Set-Content -Encoding UTF8 $cfgIn
}

# Configure + build (dynamic triplet so PCL/VTK viewer works)
$build = ".\build\vs2022-x64"
cmake -S . -B $build `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="$toolchain" `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_VIEWER=ON

cmake --build $build --config Release --parallel `
  --target minimal_encode `
  --target pcd_rmse `
  --target pcd_chamfer `
  --target pcc_viewer

# Sample data
New-Item -ItemType Directory -Force -Path .\data | Out-Null
$pcd = ".\data\table_scene_lms400.pcd"
if (!(Test-Path $pcd)) {
  Invoke-WebRequest -Uri "https://github.com/PointCloudLibrary/data/raw/master/tutorials/table_scene_lms400.pcd" `
                    -OutFile $pcd
}

# Encode/Decode at q=14 + sizes/metrics
$E  = Join-Path $build 'Release\minimal_encode.exe'
$RM = Join-Path $build 'Release\pcd_rmse.exe'
$CH = Join-Path $build 'Release\pcd_chamfer.exe'
$drc = ".\table_q14.drc"; $rec = ".\table_q14.pcd"

& $E encode -i $pcd -o $drc --qpos-bits 14
& $E decode -i $drc -o $rec

$orig  = (Get-Item $pcd).Length
$comp  = (Get-Item $drc).Length
$ratio = [math]::Round($comp / $orig, 4)
$Npts  = 460400 # point count for table_scene_lms400
$bpp   = [math]::Round(($comp * 8.0) / $Npts, 3)

$rmseVal = ([regex]::Match(((& $RM $pcd $rec 2>&1) -join "`n"), 'RMSE.*?:\s*([0-9.eE+-]+)')).Groups[1].Value
$chVal   = ([regex]::Match(((& $CH $pcd $rec 2>&1) -join "`n"), 'Chamfer RMSE:\s*([0-9.eE+-]+)')).Groups[1].Value

"`n=== q=14 summary ==="
"Original : $orig bytes"
"Compressed: $comp bytes"
"Ratio    : $ratio x"
"Bits/pt  : $bpp"
"RMSE     : $rmseVal"
"Chamfer  : $chVal"

# Rate–Distortion sweep q=8..16 -> rd_table_scene.csv
$rows = foreach ($q in 8..16) {
  $qdrc = ".\table_q$q.drc"; $qrec = ".\table_q$q.pcd"
  & $E encode -i $pcd -o $qdrc --qpos-bits $q | Out-Null
  & $E decode -i $qdrc -o $qrec | Out-Null
  $bytes = (Get-Item $qdrc).Length
  $rat   = [math]::Round($bytes / $orig, 6)
  $qbpp  = [math]::Round(($bytes * 8.0) / $Npts, 3)
  $co    = & $CH $pcd $qrec 2>&1
  $cm    = ([regex]::Match(($co -join "`n"), 'Chamfer RMSE:\s*([0-9.eE+-]+)')).Groups[1].Value
  [pscustomobject]@{ qpos_bits=$q; drc_bytes=$bytes; ratio=$rat; bpp=$qbpp; chamfer_m=[double]$cm }
}
$csv = ".\rd_table_scene.csv"
$rows | Tee-Object -Variable RD | Format-Table
$RD | ConvertTo-Csv -NoTypeInformation | Set-Content $csv
"`nWrote $csv"

# Launch viewer (side-by-side)
$env:Path = "$repo\build\vs2022-x64\vcpkg_installed\x64-windows\bin;$env:Path"
$viewer = Join-Path $build 'Release\pcc_viewer.exe'
Start-Process -FilePath $viewer -ArgumentList "`"$pcd`"","`"$rec`""
