#!/bin/bash

# Usage: ./run-specific.sh a-list-of-names

for configname in $*;
do
    mkdir ./results/${configname};
    echo ${configname};
    for inputname in `ls ./topos/`;
    do
        ./${configname} ./topos/${inputname} 2> ./results/${configname}/${inputname}-out > ./results/${configname}/${inputname}-traces;
    done
done
