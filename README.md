# Lifted Multi-agent A*

This repository contains the code for my master's [thesis](Thesis.pdf) in computer science at the University of Münster, Germany, titled:
```
Lifting Multi-agent A* - Implementing a New Optimisation Problem in Isomorphic DecPOMDPs: How Many Agents Do We Need?
```

Requires the MADP Toolbox (https://github.com/MADPToolbox/MADP).

Updated MADP installation instructions for Ubuntu 20.04 LTS can be found in the [installation notes](madp_installation_notes.txt).


## Compiling

```
g++ -o <NAME> <FILE>.cpp -I <PATHTOMADP>/include/madp -L <PATHTOMADP>/lib -lMADP
```
\<PATHTOMADP> is /usr/local by default, but can be changed during the MADP installation if desired.

## Running

```
./<NAME> <PROBLEM>.dpomdp -o<U_MIN> [OPTIONS]
```
where <U_MIN> is the desired utility for which to find n*. For a description of the .dpomdp file syntax, please refer to [example.dpomdp](example.dpomdp), or the MADP Toolbox documentation.
