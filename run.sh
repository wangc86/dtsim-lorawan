#!/bin/bash

# Usage: ./run.sh

for configname in `ls s*`;
#for configname in s*d*f*j*;
do
    ./run-specific.sh ${configname}
done
