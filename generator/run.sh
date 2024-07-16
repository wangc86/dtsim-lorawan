#!/bin/bash

#echo "./a.out ../topos-no-phase/d$1-$2.txt"

for i in 3 9 15;
do
    for j in `seq 1 1 10`;
    do
        #for k in `seq 1 1 10`;
        for k in 1;
        do
            echo "../topos/${i}-${j}-${k}";
            ./a.out ../topos-no-phase/d${i}-${j} > ../topos/${i}-${j}-${k};
            # sleep to avoid using the same time seed for pseudonumber generator
            sleep 1
        done
    done
done
