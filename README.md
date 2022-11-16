# Lifted Multi-agent A*

Requires the MADP Toolbox (https://github.com/MADPToolbox/MADP)

Updated MADP installation instructions for Ubuntu 20.04 LTS can be found in the [installation notes](madp_installation_notes.txt).


## Compiling

```
g++ -o <NAME> <FILE>.cpp -I <PATHTOMADP>/include/madp -L <PATHTOMADP>/lib -lMADP
```
default \<PATHTOMADP> = /usr/local

## Running

```
./<NAME> <PROBLEM>.dpomdp -o<U_MIN> [OPTIONS]
```
where <U_MIN> is the desired utility for which to find n*
