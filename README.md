# pointcloud_codec

A research-friendly **LiDAR point-cloud codec** that combines **position quantization** with **Google Draco** entropy coding. It ships with a tiny encoder/decoder, a PCL/VTK viewer for **side-by-side** inspection (original vs reconstructed), and metric tools for **RMSE** and **Chamfer RMSE** so you can study rate–distortion behavior on real data.

---

## What this project does

- **Quantize XYZ** positions to an integer grid (`qpos_bits` per axis) and compress with **Draco**.
- **Decode** to PCD for downstream tooling and **visualize** original vs reconstructed point clouds.
- **Measure distortion** with:
  - **Per-index RMSE** (simple RMS over corresponding samples)
  - **Symmetric Chamfer RMSE** (nearest-neighbor in both directions, robust to small reordering)
- **Sweep qpos_bits** to export **rate–distortion tables** (bytes, ratio, bits/point, Chamfer).

---

## Components

- **Library (`pcc`)** – quantisation, AABB helpers, and glue to Draco
- **CLI tools**
  - `minimal_encode` – PCD ⇄ DRC encoder/decoder
  - `pcd_rmse` – per-index RMSE between two PCDs
  - `pcd_chamfer` – symmetric Chamfer RMSE between two PCDs
- **Viewer** – `pcc_viewer` (PCL/VTK) renders original (left) and reconstructed (right)

---

## Example results (PCL sample: `table_scene_lms400.pcd`)

**One operating point (qpos_bits = 14)**

- Points: **460,400**
- Original PCD: **5,649,007 bytes**
- Compressed DRC: **956,270 bytes** → **0.169×** the original
- Bits per point: **16.616 bpp**
- RMSE (per-index): **0.9095**  
- Chamfer RMSE (NN-based): **6.27×10⁻⁵**  
*(Units follow the dataset; PCL “table scene” is in meters.)*

**Rate–Distortion sweep (q = 8…16)**

| qpos_bits | DRC bytes | Ratio (comp/orig) | Bits/pt | Chamfer RMSE |
|:---------:|----------:|------------------:|--------:|-------------:|
| 8  | 55,198   | 0.009771 | 0.959  | 3.214e-03 |
| 9  | 135,545  | 0.023994 | 2.355  | 1.791e-03 |
| 10 | 275,408  | 0.048753 | 4.786  | 9.816e-04 |
| 11 | 442,014  | 0.078246 | 7.681  | 4.991e-04 |
| 12 | 612,664  | 0.108455 | 10.646 | 2.501e-04 |
| 13 | 784,200  | 0.138821 | 13.626 | 1.253e-04 |
| 14 | 956,270  | 0.169281 | 16.616 | 6.268e-05 |
| 15 | 1,128,719| 0.199808 | 19.613 | 3.135e-05 |
| 16 | 1,301,269| 0.230354 | 22.611 | 1.568e-05 |

*Takeaway:* increasing `qpos_bits` improves geometry fidelity (lower Chamfer) at a predictable cost in bitrate.

---

## Design notes

- Quantization uses a dataset AABB and clamps tiny extents to avoid divide-by-zero.
- Distortion is reported two ways because real pipelines sometimes reorder points:
  - **Per-index RMSE** is quick and intuitive.
  - **Chamfer RMSE** is insensitive to permutation and better reflects visual quality.
- The viewer highlights differences visually and is helpful for qualitative checks alongside numeric metrics.

---

## Technologies

- **C++20**, **CMake**
- **Draco** (geometry compression), **Eigen** (math)
- **PCL / VTK** (I/O and visualization)
- **vcpkg** (dependency management)

---



