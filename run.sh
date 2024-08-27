#!/bin/bash

# Usage: ./run.sh  This will run the all configurations; use run-specific.sh to run some of them only.

for configname in `ls s*`;
#for configname in s*d*f*j*;
do
    ./run-specific.sh ${configname}
done
