<div align="center">

# Systolic PE Array Accelerator for 2D Convolution on FPGA

### Design · Multi-Precision Analysis · Real-World Sobel Edge Detection on KITTI Image

[![Language](https://img.shields.io/badge/Language-Verilog%20HDL-blue?style=flat-square)](https://en.wikipedia.org/wiki/Verilog)
[![Target](https://img.shields.io/badge/Target-Xilinx%20FPGA-orange?style=flat-square)](https://www.xilinx.com/)
[![Clock](https://img.shields.io/badge/Clock-100%20MHz-green?style=flat-square)](#)
[![Precision](https://img.shields.io/badge/Precision-Q8.8%20|%20Q12.4%20|%20Q12.8%20|%20Q12.12-purple?style=flat-square)](#)
[![Architecture](https://img.shields.io/badge/Architecture-Output%20Stationary-red?style=flat-square)](#)

</div>



## Project Overview

This project implements a parameterized **4×4 Systolic Processing Element (PE) Array** in Verilog HDL for hardware-accelerated 2D convolution on FPGA. The work is structured in three progressive phases:

| Phase | Description |
|-------|-------------|
| **Phase 1** | Design and verify a 4×4 Output Stationary systolic array for Q8.8 fixed-point arithmetic, validated cycle-by-cycle against a Python golden reference model |
| **Phase 2** | Parameterize the architecture across four fixed-point precision formats (Q8.8, Q12.4, Q12.8, Q12.12) and analyze timing, resource, and power trade-offs |
| **Phase 3** | Deploy the array for real-world **Sobel edge detection** on a 640×640 KITTI autonomous driving image using a tile-based BRAM pipeline |

The final system processes an entire 640×640 image through the 4×4 PE array by tiling it into sequential 6×6 windows, loading each tile through BRAM into the systolic array, collecting the 4×4 convolution result, and reassembling the complete output feature map — demonstrating a full end-to-end FPGA-accelerated image processing pipeline.

---

## Phase 1 — 4×4 Systolic PE Array Design (Q8.8)

### Architecture

The array implements an **Output Stationary (OS)** dataflow. Each of the 16 PEs is permanently assigned to one output pixel of the feature map and accumulates its own partial sum locally until computation completes. This eliminates intermediate result movement, making OS ideal for convolution workloads.

```
                    W-Col0      W-Col1      W-Col2      W-Col3
                  (No delay)  (Delay: 4)  (Delay: 8)  (Delay:12)
                      │           │           │           │
                      ▼           ▼           ▼           ▼
Row0 (No delay) ──►[PE[0,0]]──►[PE[0,1]]──►[PE[0,2]]──►[PE[0,3]]
                      │           │           │           │
Row1 (Delay: 1) ──►[PE[1,0]]──►[PE[1,1]]──►[PE[1,2]]──►[PE[1,3]]
                      │           │           │           │
Row2 (Delay: 2) ──►[PE[2,0]]──►[PE[2,1]]──►[PE[2,2]]──►[PE[2,3]]
                      │           │           │           │
Row3 (Delay: 3) ──►[PE[3,0]]──►[PE[3,1]]──►[PE[3,2]]──►[PE[3,3]]
```

**Core PE MAC operation:**
```
Acc[n] = Acc[n-1] + (Pixel × Weight)
```

Each PE also passes its pixel and weight values to its right and downward neighbors respectively, propagating data through the array.

### Data Flow and Input Staggering

The system converts a **6×6 input image** convolved with a **3×3 kernel** into a **4×4 output feature map**. Inputs are staggered (delayed in time) so that the correct pixel always meets the correct weight at the correct PE on every clock cycle.

**Input Image (6×6):**
```
 1   2   3   4   5   6
 7   8   9  10  11  12
13  14  15  16  17  18
19  20  21  22  23  24
25  26  27  28  29  30
31  32  33  34  35  36
```

**Kernel (3×3):**
```
1  2  3
4  5  6
7  8  9
```

**Row Input Staggering (Image Pixels streamed horizontally):**

| Row | Delay     | Input Sequence                              |
|-----|-----------|---------------------------------------------|
| 0   | 0 cycles  | `[1, 7, 13, 2, 8, 14, 3, 9, 15 ...]`       |
| 1   | 1 cycle   | `[0, 7, 13, 19, 8, 14, 20 ...]`             |
| 2   | 2 cycles  | `[0, 0, 13, 19, 25, 14, 20 ...]`            |
| 3   | 3 cycles  | `[0, 0, 0, 19, 25, 31, 20 ...]`             |

**Column Input Staggering (Kernel weights streamed vertically):**

| Column | Delay     | Sequence                                   |
|--------|-----------|--------------------------------------------|
| 0      | 0 cycles  | `[W1, W4, W7, W2, W5, W8, W3, W6, W9 ...]` |
| 1      | 4 cycles  | Delayed copy of Column 0                   |
| 2      | 8 cycles  | Delayed copy of Column 0                   |
| 3      | 12 cycles | Delayed copy of Column 0                   |

### Python Golden Reference Model

Before hardware implementation, the complete system was modeled in Python using a custom **Q8.8 Fixed-Point class** that mirrors hardware arithmetic exactly — including truncation behavior and scaling.

**Q8.8 Format:**
- 16-bit total: 8 integer bits + 8 fractional bits
- Scaling factor: 2⁸ = 256  →  `Real Value = Raw Integer ÷ 256`
- Fractional resolution: `2⁻⁸ = 0.0039`
- Representable range: `−128` to `+127.996`

The script simulates each clock cycle, modeling the exact staggered data arrival, PE accumulation schedule, and Q8.8 rounding. The final 4×4 output is compared against a NumPy double-precision floating-point reference.

**Python Simulation Output:**
```
4x4 PE ARRAY SYSTOLIC CONVOLUTION (6x6 image, 3x3 kernel)
Q8.8 Fixed-Point Format
══════════════════════════════════════════════════════

Input Image (6x6):
[[ 1,  2,  3,  4,  5,  6],
 [ 7,  8,  9, 10, 11, 12],
 [13, 14, 15, 16, 17, 18],
 [19, 20, 21, 22, 23, 24],
 [25, 26, 27, 28, 29, 30],
 [31, 32, 33, 34, 35, 36]]

Weight Kernel (3x3):
[[1, 2, 3],
 [4, 5, 6],
 [7, 8, 9]]

══════════════════════════════════════════════════════
FINAL OUTPUT FEATURE MAP (4x4) — Q8.8 Format:
[[ 474,  519,  564,  609],
 [ 744,  789,  834,  879],
 [1014, 1059, 1104, 1149],
 [1284, 1329, 1374, 1419]]

VERIFICATION (Standard Floating-Point Convolution):
[[ 474,  519,  564,  609],
 [ 744,  789,  834,  879],
 [1014, 1059, 1104, 1149],
 [1284, 1329, 1374, 1419]]

✓ MATCH: Q8.8 systolic output matches floating-point convolution!
Maximum error: 0.0000
══════════════════════════════════════════════════════
```

### Verilog RTL Implementation

The hardware design is modular, with each file handling a specific layer of the hierarchy:

| Module | File | Description |
|--------|------|-------------|
| Testbench | `tb_systolic_conv.v` | Generates clock, reset, and drives staggered input stimuli |
| Top Level | `Systolic_Conv_Top.v` | Integrates PE array, BRAM buffers, and output write-back logic |
| PE Array | `PE_Array_4x4.v` | Instantiates the 4×4 PE grid and manages horizontal/vertical data routing |
| PE Core | `PE.v` | MAC unit — Multiplier + Adder + Accumulator register |
| Input Buffer | `BRAM_Buffer.v` | Block RAM for pixel and kernel weight storage |
| Line Buffer | `Line_Buffer.v` | Shift-register-based horizontal pixel streaming |

**PE.v — Core Accumulation Logic (Q8.8):**
```verilog
always @(posedge clk or posedge rst) begin
    if (rst) begin
        acc        <= 32'd0;
        pixel_out  <= 16'd0;
        weight_out <= 16'd0;
    end else begin
        acc        <= acc + (pixel_in * weight_in);   // Q8.8 MAC
        pixel_out  <= pixel_in;                        // Propagate right
        weight_out <= weight_in;                       // Propagate down
    end
end
```

### Simulation and Verification

The Verilog behavioral simulation was run in Xilinx Vivado. All 16 PEs compute in parallel across the staggered input schedule.

**Key signal verification:**

| Signal | Raw Q8.8 Value | Decoded (÷ 256) | Python Reference | Status |
|--------|----------------|-----------------|-----------------|--------|
| `out00` | 121344 | **474.00** | 474.00 | Exact |
| `out20` | 259584 | **1013.99** | 1014.00 | Quantization only |
| `out33` | 363264 | **1419.00** | 1419.00 | Exact |

**Full Verilog Console Output:**
<p align="center">
  <img src="results/phase_1/console%20output.png" width="200">
</p>

<p align="center">
  <b>Fig. 1. Console Output </b>
</p>

<p align="center">
  <img src="results/phase_1/Waveform.png" width="600">
</p>

<p align="center">
  <b>Fig. 2. Timing Waveform </b>
</p>


**The Verilog implementation is functionally identical to the Python golden reference model.**

### I/O Bottleneck and Output BRAM Optimization

**Problem — Initial Design:**

The first implementation exposed all 16 PE outputs (32 bits × 16 outputs = **512 output pins**) directly to top-level ports.

| Resource | Initial | % Used |
|----------|---------|--------|
| LUT  | 547  | 0.86% |
| DSP  | 16   | 6.67% |
| BRAM | 4    | 2.96% |
| **IO**   | **569**  | **270.95% ❌** |

270% I/O utilization is physically impossible on any real FPGA package.

**Solution — Immediate Write-Back Output BRAM:**

An output dual-port Block RAM was integrated. As soon as each PE completes its convolution and its result is valid, it immediately asserts a write-enable and stores its result in BRAM — without waiting for the full array to finish. The host then reads 16 results sequentially over a narrow 32-bit bus.

This decouples the internal parallel computation from the external readout interface.

**Optimized Resource Report:**

| Resource | Utilization | Available | % Used |
|----------|-------------|-----------|--------|
| LUT      | 710         | 63,400    | **1.12%** |
| FF       | 956         | 126,800   | **0.75%** |
| BRAM     | 4.5         | 135       | **3.33%** |
| DSP      | 16          | 240       | **6.67%** |
| IO       | 69          | 210       | **32.86% ✅** |

I/O utilization reduced from **271% → 32.86%** with less than 0.3% additional logic overhead.

### Timing and Power Analysis

**Constraints:** 100 MHz system clock (10 ns period), 3.3V configuration voltage.  
DRC waivers applied to downgrade "missing physical pin" errors to warnings for post-route analysis.

**Design Timing Summary:**
<p align="center">
  <img src="results/phase_1/Timing_report.png" width="500">
</p>
<p align="center">
  <b>Fig. 3. Timing Analysis </b>
</p>

The positive WNS confirms the critical path (accumulator carry chain inside PEs) is well within the clock period, with headroom for higher frequency operation.

**On-Chip Power Summary:**

<p align="center">
  <img src="results/phase_1/Power_report.png" width="500">
</p>
<p align="center">
  <b>Fig. 4. Power Analysis </b>
</p>
The core computational logic (DSPs and LUTs) is extremely energy-efficient. Data movement (I/O and BRAM) accounts for the majority of active power.

---

## Phase 2 — Multi-Precision Architecture Analysis

The Verilog PE architecture was parameterized to evaluate four fixed-point formats, characterizing the hardware cost of increasing arithmetic precision.

| Format | Total Bits | Integer Bits | Fractional Bits | Resolution | Max Value |
|--------|-----------|-------------|----------------|-----------|-----------|
| Q8.8   | 16-bit | 8  | 8  | 2⁻⁸  = 0.0039   | ±128  |
| Q12.4  | 16-bit | 12 | 4  | 2⁻⁴  = 0.0625   | ±2048 |
| Q12.8  | 20-bit | 12 | 8  | 2⁻⁸  = 0.0039   | ±2048 |
| Q12.12 | 24-bit | 12 | 12 | 2⁻¹² = 0.00024  | ±2048 |

---

### Q12.4 — High Dynamic Range (16-bit)

Remaps the 16-bit datapath to 12 integer bits and 4 fractional bits. Provides a significantly higher integer range (±2048 vs ±128) at the cost of coarser fractional resolution. Suitable for accumulations with large integer values where sub-unit precision is not critical.

<table align="center">
  <tr>
    <td align="center">
      <b>Resource Utilization — Graph View</b><br><br>
      <img src="results/phase_2/12.4/Utilization_graph.png" width="400">
    </td>
    <td align="center">
      <b>Resource Utilization — Table View</b><br><br>
      <img src="results/phase_2/12.4/Utilization_table.png" width="400">
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>Timing Analysis — 100 MHz Constraint</b><br><br>
      <img src="results/phase_2/12.4/Timing_report.png" width="500">
    </td>
    <td align="center">
      <b>On-Chip Power Report</b><br><br>
      <img src="results/phase_2/12.4/Power_report.png" width="400">
    </td>
  </tr>
</table>

---

### Q12.8 — Extended Precision (20-bit)

Expands the total datapath to 20 bits, gaining both high integer range (12 bits) and fine fractional precision (8 bits). The wider datapath requires wider interconnects and multiplexers at the BRAM interface, increasing logic utilization noticeably.

<table align="center">
  <tr>
    <td align="center">
      <b>Resource Utilization — Graph View</b><br><br>
      <img src="results/phase_2/12.8/Utilization_graph.png" width="400">
    </td>
    <td align="center">
      <b>Resource Utilization — Table View</b><br><br>
      <img src="results/phase_2/12.8/Utilization_table.png" width="400">
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>Timing Analysis — 100 MHz Constraint</b><br><br>
      <img src="results/phase_2/12.8/Timing_report.png" width="500">
    </td>
    <td align="center">
      <b>On-Chip Power Report</b><br><br>
      <img src="results/phase_2/12.8/Power_report.png" width="400">
    </td>
  </tr>
</table>

**Resource Utilization:** LUT usage tripled compared to 16-bit formats due to the 20-bit BRAM interface logic. DSP count remains 16 as, the 20-bit multiplication still fits within a single DSP48E cascade.
**Timing:** WNS = **+0.232 ns** — tightest margin of all formats, caused by wider carry propagation in the 20-bit accumulator. Still meets 100 MHz cleanly.  
**Power:** Total = **0.132 W** — slight increase from higher bus switching activity.

---

### Q12.12 — Maximum Precision (24-bit)

Maximizes fractional resolution at 2⁻¹² = 0.00024 for applications like gradient computation, high-fidelity image filtering, or neural network inference layers requiring fine granularity. This format places the highest demand on FPGA resources.

**Critical Architectural Impact — DSP Cascading:**

Standard Xilinx DSP48E slices support multiplier inputs up to 18×25 bits. Since Q12.12 requires **24×24-bit multiplication**, the synthesis tool automatically **cascades two DSP48E slices per PE** to perform the wider operation — doubling DSP usage from 16 to **32 blocks**.

<table align="center">
  <tr>
    <td align="center">
      <b>Resource Utilization — Graph View</b><br><br>
      <img src="results/phase_2/12.12/Before_timing_met/Utilization_graph.png" width="400">
    </td>
    <td align="center">
      <b>Resource Utilization — Table View</b><br><br>
      <img src="results/phase_2/12.12/Before_timing_met/Utilization_Table.png" width="400">
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>Timing Analysis — Before Optimization</b><br><br>
      <img src="results/phase_2/12.12/Before_timing_met/Timing_report.png" width="500">
    </td>
    <td align="center">
      <b>On-Chip Power Report</b><br><br>
      <img src="results/phase_2/12.12/Before_timing_met/power_report.png" width="400">
    </td>
  </tr>
</table>

**Timing:** The 24-bit × 24-bit multiplication followed by 48-bit accumulation in a single clock cycle exceeded the 10 ns constraint by **12 picoseconds**. The cascaded DSP routing delay between the two DSP48E slices per PE is the root cause.

**Power:** Total = **0.137 W** — highest of all formats, driven by doubled DSP toggle rate and wider buses.


### Q12.12 Optimized — 2-Stage Pipelined MAC

To resolve the timing violation, the PE was modified from a **single-cycle combinational MAC** to a **2-stage pipelined MAC**.

**Modification — Adding a Pipeline Register:**

A register (`mult_stage_reg`) is inserted between the multiplier output and the accumulator adder input, breaking the long critical path into two balanced stages:

```
Stage 1 (Cycle N):   mult_result = pixel_in × weight_in
Stage 2 (Cycle N+1): acc = acc + mult_result
```

This adds **1 cycle of latency** to the final result — negligible in a systolic pipeline where data arrives staggered across many cycles anyway.


<table align="center">
  <tr>
    <td align="center">
      <b>Resource Utilization — Graph View</b><br><br>
      <img src="results/phase_2/12.12/After_timing_met/Utilization_graph.png" width="400">
    </td>
    <td align="center">
      <b>Resource Utilization — Table View</b><br><br>
      <img src="results/phase_2/12.12/After_timing_met/Utilization_table.png" width="400">
    </td>
  </tr>

  <tr>
    <td align="center">
      <b>Timing Analysis — After Optimization</b><br><br>
      <img src="results/phase_2/12.12/After_timing_met/Timing_report.png" width="500">
    </td>
    <td align="center">
      <b>On-Chip Power Report</b><br><br>
      <img src="results/phase_2/12.12/After_timing_met/Power_report.png" width="400">
    </td>
  </tr>
</table>

**Resource Utilization:** LUT count decreased because the synthesis tool packs separated logic more efficiently when the multiply and accumulate stages are decoupled.

**Timing:** The +1.011 ns slack is comparable to the Q8.8 baseline (+0.899 ns), confirming the pipeline fully resolves the critical path.

**Power:** Total = **0.157 W** — small increase from additional pipeline register switching.


### Consolidated Results Summary

| Metric | Q8.8 | Q12.4 | Q12.8 | Q12.12 (Pipelined) |
|--------|:----:|:-----:|:-----:|:------------------:|
| **Bit Width**     | 16-bit | 16-bit | 20-bit | 24-bit |
| **LUTs**          | 710 (1.12%) | 705 (1.11%) | 2026 (3.20%) | 990 (1.56%) |
| **Flip-Flops**    | 956 (0.75%) | 956 (0.75%) | 1188 (0.94%) | 1519 (1.20%) |
| **DSP Blocks**    | 16 (6.67%) | 16 (6.67%) | 16 (6.67%) | 32 (13.33%) |
| **BRAM**          | 4.5 (3.33%) | 4.5 (3.33%) | 5.0 (3.70%) | 5.0 (3.70%) |
| **IO Ports**      | 69 (32.86%) | 69 (32.86%) | 81 (38.57%) | 93 (44.29%) |
| **WNS**           | +0.899 ns | +1.441 ns | +0.232 ns | +1.011 ns |
| **Total Power**   | 0.119 W | 0.119 W | 0.132 W | 0.157 W |
| **Dynamic Power** | 0.028 W | 0.028 W | 0.041 W | 0.066 W |

**Key Insights:**
- Q8.8 and Q12.4 are physically identical in hardware, have same 16-bit datapath, differing only in fixed-point interpretation.
- Moving from 16-bit to 20-bit (Q12.8) triples LUT usage due to wider BRAM mux/demux logic.
- Moving to 24-bit (Q12.12) doubles DSP blocks due to automatic DSP cascade for wide multiplication, and requires pipelining to meet timing.
- All four formats operate at **100 MHz** with positive timing slack post-implementation.
- Even the most resource-intensive format (Q12.12 pipelined) consumes only **0.157 W** which is well-suited for edge AI inference devices.


## Phase 3 — Sobel Edge Detection on 640×640 KITTI Image

### System Architecture

The validated 4×4 PE array is deployed for practical **Sobel edge detection** on a 640×640 grayscale KITTI autonomous driving scene image. Since the PE array processes 6×6 input windows at a time, the full image is processed through a systematic **tile-based BRAM pipeline**.




### Tiling Strategy

The 640×640 image is scanned with a **6×6 sliding window** at stride 1 to produce a valid convolution output:

| Parameter | Value |
|-----------|-------|
| Input image size | 640 × 640 pixels |
| Tile (window) size | 6 × 6 pixels |
| PE array output per tile | 4 × 4 pixels |
| Output feature map size | 638 × 638 pixels |
| Total tiles processed | 638 × 638 = **407,044 tiles** |

Each 6×6 tile is streamed into the PE array using the standard staggered schedule:
- Row delays: 0, 1, 2, 3 cycles
- Column delays: 0, 4, 8, 12 cycles

### BRAM-Based Data Pipeline

Each tile is processed through the following pipeline stages:

```
1. Load 6×6 pixel tile → Pixel BRAM
2. Load 3×3 Sobel kernel → Weight BRAM
3. Assert start signal → PE array begins staggered computation
4. Wait for done signal → all 16 PEs have valid accumulated results
5. Read 16 results from Output BRAM
6. Write 4×4 result block to correct (row, col) position in output buffer
7. Advance tile pointer → return to Step 1
```

**Sobel Kernels:**

```
Sobel-X (Horizontal Gradient):     Sobel-Y (Vertical Gradient):
 -1   0  +1                          -1  -2  -1
 -2   0  +2                           0   0   0
 -1   0  +1                          +1  +2  +1
```

Final edge magnitude at each pixel: `|Gx| + |Gy|`  (L1 norm approximation)

The Sobel-X and Sobel-Y passes are run as two separate sweeps through the full image, and the results are combined to produce the final edge map.

### Output Results Across Precision Formats

The systolic array was applied to the KITTI image with three precision formats. The results show a clear and progressive improvement in edge detection quality as fractional precision increases.

<p align="center">
  <img src="results/phase_3/Verilog_output_comparision_page-0001.jpg" width="500"><br>
  <b>Figure: Verilog Outputs for different Dataformats </b>
</p>

**Precision vs. Edge Quality Observations:**

| Format | Edge Detail | Notes |
|--------|------------|-------|
| Q8.8   | Low — dominant edges only | Limited fractional bits cause small gradient values to be quantized to zero. Road boundaries and building outlines are captured; fine textures and foliage are lost. |
| Q12.12 | High — fine structural edges | Improved fractional precision recovers subtle gradients. Window frames, vegetation edges, and pavement markings become visible. |
| Q16.16 | Highest — all structural and textural detail | All edges preserved with maximum fidelity. Best result for downstream perception tasks like lane detection or object boundary extraction. |



## Dependencies

| Tool | Version | Purpose |
|------|---------|---------|
| Xilinx Vivado | 2020.x or later | Synthesis, P&R, STA, Power Analysis |
| Python | 3.8+ | Golden reference model and tile pipeline |
| NumPy | Any | Matrix operations in reference model |
| Pillow (PIL) | Any | Image loading/saving for KITTI pipeline |


## Contributors
- **[Madhan Lankalapalli](https://github.com/your-username)** 
- **[Prasanth Behara](https://github.com/your-username)** 
- **[Rupa Lakshmi Koram](https://github.com/your-username)** 


<div align="center">

**Designed and Implemented at RGUKT**  
*Design and Implementation of a 4×4 Systolic PE Array for 2D Convolution*

</div>
