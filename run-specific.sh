#!/bin/bash

# Usage: ./run.sh name

for configname in $1;
do
    mkdir ./results/${configname};
    for inputname in `ls ./topos/`;
    do
        echo ${configname};
        ./${configname} ./topos/${inputname} 2> ./results/${configname}/${inputname}-out;
    done
done
