# IOPDDL 2025 Solution

Stephan Spengler, Uppsala University (stephan.spengler@it.uu.se)\
Samuel Grahn, Uppsala University (samuel.grahn@it.uu.se)

This repository hosts our solution for the [**The ASPLOS 2025 / EuroSys 2025 Contest on Intra-Operator Parallelism for Distributed Deep Learning**](https://github.com/asplos-contest/2025/blob/main/IOPDDL.md), awarded with the third place 🥉.

## Example Usage

```bash
git clone --recursive https://github.com/StephanSpengler/iopddl-submission.git
mkdir iopddl-submission/build && cd iopddl-submission/build && cmake .. && make
./iopddl ../example.json 10
```

## Benchmarks

Benchmarks can be found [**here**](https://github.com/google/iopddl/tree/main/benchmarks).
