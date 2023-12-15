# Anytime Multi-Agent Path Finding

This software is based on the latest [MAPF-LNS implementation](https://github.com/Jiaoyang-Li/MAPF-LNS2) from [1].

## Featured Algorithm
- BALANCE: **B**andit-based **A**daptive **LA**rge **N**eighborhood search **C**ombined with **E**xploration [2]

## Usage
The code requires the external libraries [`BOOST 1.81.0`](https://www.boost.org/) and [`Eigen 3.3`](https://eigen.tuxfamily.org/), and [`CMake`](https://cmake.org) for building the code. 
    
After installing both libraries go to the root folder of this repository and run the following commands: 
```shell script
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
```

Run the code with:
```
./balance -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test -k 350 -t 300 --outputPaths=paths.txt --banditAlgo=Thompson --neighborCandidateSizes=5 --seed=0
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- o: the output file name (no need for file extension)
- k: the number of agents
- t: the runtime limit
- outputPaths: the output file that contains the paths
- banditAlgo: the multi-armed bandit algorithm used in BALANCE
- neighborCandidateSizes: the number of neighborhood size options
- seed: the random seed

You can find more details and explanations for all parameters with:
```
./balance --help
```

We provide example instance files `random-32-32-20.map` and `random-32-32-20-random-1.scen` in the repo. More instances can be downloaded from the [MovingAI MAPF benchmark](https://movingai.com/benchmarks/mapf/index.html).

## Credits

The software is mainly based on code developed by Jiaoyang Li and Zhe Chen in [MAPF-LNS2](https://github.com/Jiaoyang-Li/MAPF-LNS2).

The rule-based MAPF solvers (i.e., PPS, PIBT, and winPIBT) inside the software were borrowed from 
https://github.com/Kei18/pibt/tree/v1.3

BALANCE is released under USC–Research License. See license.txt for further details.

## References

- [1] J. Li et al. *"MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search"*. AAAI 2022.
- [2] T. Phan et al. *"Adaptive Anytime Multi-Agent Path Finding using Bandit-Based Large Neighborhood Search"*. AAAI 2024.