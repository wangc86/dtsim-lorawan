#!/bin/bash

# Usage:
# First, compile executables s*d*f*j*. See README.md for the naming convention.
# Then configname refers to each executable name.
# Run ./run-specific.sh a-list-of-names (note: $* refers to the list of command line arguments other than the name of the command)

#TODO: update this to experiment on the effect of attacker's moving speed
jammerspeed=6

for configname in $*;
do
    mkdir ./results/${configname};
    echo ${configname} "; jammer speed =" ${jammerspeed};
    for inputname in `ls ./topos/`;
    do
        ./${configname} ./topos/${inputname} ${jammerspeed} 2> ./results/${configname}/${inputname}-out > ./results/${configname}/${inputname}-traces;
    done
done
