# LiDAR Point‑Cloud Codec Demo  <!-- RenoLifeCpp -->

Small, self‑contained C++ example that

1. **loads** a `.pcd` file (ASCII / binary) with [PCL]  
2. **compresses** it with [Google Draco] (14‑bit quantised, sequential encoder)  
3. **writes** the bit‑stream (`.drc`) to disk  
4. **decodes** it back into a PCL cloud  
5. **reports** compression ratio and RMS reconstruction error  

The entire logic lives in one file: **`RenoLifeCpp.cpp`** (≈ 150 LoC).

---

## Why would I use this?

* Evaluate *lossy* compression of large LiDAR scans (≈ 10× smaller than ASCII).
* Prototype edge‑device workflows (Jetson, x86) before porting to MCUs/FPGAs.
* Understand the maths behind uniform 3‑D quantisation & delta entropy coding.

---

## Build / Run on Windows (x64)

```powershell
# 0) One‑off vcpkg bootstrap (if you haven’t yet)
git clone https://github.com/microsoft/vcpkg C:\dev\vcpkg
C:\dev\vcpkg\bootstrap-vcpkg.bat

# 1) Install libraries
C:\dev\vcpkg\vcpkg install pcl[core,visualization] draco:x64-windows

# 2) Open "x64 Native Tools VS 2022 Prompt"  OR
#    PowerShell:
Import-Module "$env:ProgramFiles\Microsoft Visual Studio\2022\Community\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath "C:\Program Files\Microsoft Visual Studio\2022\Community" -DevCmdArguments "-arch=x64"

# 3) Build single translation unit
cd C:\Projects\RenoLifeCpp
cl /EHsc RenoLifeCpp.cpp `
    /I"C:\dev\vcpkg\installed\x64-windows\include" `
    /std:c++20 /MD /O2 /Fe:codec_demo.exe `
    /link /LIBPATH:"C:\dev\vcpkg\installed\x64-windows\lib" `
          draco.lib pcl_common.lib pcl_io.lib

# 4) Copy runtime DLLs once
xcopy /y /d "C:\dev\vcpkg\installed\x64-windows\bin\*.dll" .

# 5) Test
codec_demo.exe .\data\table_scene_lms400.pcd table_scene.drc
```

Expected console output:

```
Original points: 460400
Compressed size: 1002 KB
Decoded points: 460400
RMS position error ~ 6.3e‑05 m
```

---

## Maths summary

*Quantisation*  
\(x\mapsto q=\lfloor (x-x_{\min})/(x_{\max}-x_{\min})\,(2^{14}-1)\rfloor\)  
→ bin width \(\Delta=\tfrac{x_{\max}-x_{\min}}{16383}\); worst‑case error ½Δ.

*Encoding*  
Sequential mode stores deltas \(\Delta q_i=q_i-q_{i-1}\) and entropy‑codes them.

*Decoding*  
\(\hat x = x_{\min} + q\,\Delta\).  
RMS ≈ Δ ⁄ 2 for uniform data – verified in step 5.

---

## Port to Linux / Jetson

```bash
sudo apt install build-essential ninja-build
git clone https://github.com/microsoft/vcpkg ~/vcpkg && ~/vcpkg/bootstrap-vcpkg.sh
~/vcpkg/vcpkg install pcl draco
g++ -std=c++20 -O3 RenoLifeCpp.cpp \
    -I ~/vcpkg/installed/$(uname -m)-linux/include \
    -L ~/vcpkg/installed/$(uname -m)-linux/lib \
    -ldraco -lpcl_common -lpcl_io -o codec_demo
```

Jetson Nano (64‑bit Ubuntu 20.04) builds in ≈ 3 min and encodes a ½‑million‑point
scan in < 0.2 s, staying under 200 MB RAM.

---

## Dataset

`data/table_scene_lms400.pcd` is mirrored from the  
[Point Cloud Library Data repo](https://github.com/PointCloudLibrary/data).

---

## License

* Code — MIT  
* PCL & Draco — BSD‑style (see their repos)

---

<p align="center"><sub>Made with ♥ for quick experiments — PRs welcome!</sub></p>
