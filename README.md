# PointCloud Codec: build, run, measure, view

$ErrorActionPreference = 'Stop'
$repo = (Get-Location).Path

# --- 0) vcpkg toolchain (adjust if your vcpkg root differs)
$env:VCPKG_ROOT = "C:\vcpkg"
$toolchain = Join-Path $env:VCPKG_ROOT 'scripts\buildsystems\vcpkg.cmake'
if (!(Test-Path $toolchain)) { throw "vcpkg toolchain not found at: $toolchain" }

# --- 1) Ensure tiny package config template exists (avoids configure_file error)
New-Item -ItemType Directory -Force -Path .\cmake | Out-Null
$cfgIn = '.\cmake\pointcloud_codecConfig.cmake.in'
if (!(Test-Path $cfgIn)) {
@'
# Minimal package config for pointcloud_codec
# (Loaded by find_package(pointcloud_codec))
include("${CMAKE_CURRENT_LIST_DIR}/pointcloud_codecTargets.cmake")
'@ | Set-Content -Encoding UTF8 $cfgIn
}

# --- 2) Configure + build (dynamic triplet so PCL/VTK viewer works)
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

# --- 3) Download sample PCD (table_scene_lms400) if missing
New-Item -ItemType Directory -Force -Path .\data | Out-Null
$pcd = ".\data\table_scene_lms400.pcd"
if (!(Test-Path $pcd)) {
  Invoke-WebRequest -Uri "https://github.com/PointCloudLibrary/data/raw/master/tutorials/table_scene_lms400.pcd" `
                    -OutFile $pcd
}

# --- 4) Encode / Decode (q=14) + sizes + basic metrics
$E   = Join-Path $build 'Release\minimal_encode.exe'
$RM  = Join-Path $build 'Release\pcd_rmse.exe'
$CH  = Join-Path $build 'Release\pcd_chamfer.exe'
$drc = ".\table_q14.drc"
$rec = ".\table_q14.pcd"

& $E encode -i $pcd -o $drc --qpos-bits 14
& $E decode -i $drc -o $rec

$orig  = (Get-Item $pcd).Length
$comp  = (Get-Item $drc).Length
$ratio = [math]::Round($comp / $orig, 4)
# Known point count for table_scene_lms400
$Npts  = 460400
$bpp   = [math]::Round(($comp * 8.0) / $Npts, 3)

$rmseOut = & $RM $pcd $rec 2>&1
$rmseVal = [regex]::Match(($rmseOut -join "`n"), 'RMSE.*?:\s*([0-9.eE+-]+)').Groups[1].Value

$chOut   = & $CH $pcd $rec 2>&1
$chVal   = [regex]::Match(($chOut -join "`n"), 'Chamfer RMSE:\s*([0-9.eE+-]+)').Groups[1].Value

"`n=== Single-point setting (q=14) ==="
"Original:  $orig bytes"
"Compressed: $comp bytes"
"Ratio:     $ratio x"
"Bits/pt:   $bpp"
"RMSE:      $rmseVal"
"Chamfer:   $chVal"

# --- 5) Rate–Distortion sweep (q=8..16) -> rd_table_scene.csv
$rows = foreach ($q in 8..16) {
  $qdrc = ".\table_q$q.drc"
  $qrec = ".\table_q$q.pcd"
  & $E encode -i $pcd -o $qdrc --qpos-bits $q | Out-Null
  & $E decode -i $qdrc -o $qrec                 | Out-Null

  $bytes = (Get-Item $qdrc).Length
  $rat   = [math]::Round($bytes / $orig, 6)
  $qbpp  = [math]::Round(($bytes * 8.0) / $Npts, 3)

  $co = & $CH $pcd $qrec 2>&1
  $cm = [regex]::Match(($co -join "`n"), 'Chamfer RMSE:\s*([0-9.eE+-]+)').Groups[1].Value

  [pscustomobject]@{
    qpos_bits = $q
    drc_bytes = $bytes
    ratio     = $rat
    bpp       = $qbpp
    chamfer_m = [double]$cm
  }
}
$rows | Tee-Object -Variable RD | Format-Table
$csv = ".\rd_table_scene.csv"
$RD  | ConvertTo-Csv -NoTypeInformation | Set-Content $csv
"`nWrote $csv"

# --- 6) Launch viewer (side-by-side)
# Ensure vcpkg DLLs (PCL/VTK) are on PATH for runtime
$env:Path = "$repo\build\vs2022-x64\vcpkg_installed\x64-windows\bin;$env:Path"
$viewer = Join-Path $build 'Release\pcc_viewer.exe'
"`nOpening viewer (close the window to return)..."
Start-Process -FilePath $viewer -ArgumentList "`"$pcd`"","`"$rec`""

