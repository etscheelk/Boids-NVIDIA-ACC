#!/bin/bash

num_trials=1
if [[ "$1" ]]
then
    num_trials="$1"
fi


max_trials=16
if [[ $num_trials -ge $max_trials ]]
then
    num_trials=$max_trials
fi

bin="../tsglBoidsGPU"
printf "Start of GPU tests %s " "$bin"; date; # nvaccelinfo;
printf "\n\n"

boidsCounts=(128 256 512 1024 2048 4096 8192)

for boidCount in "${boidsCounts[@]}"
do
    printf "numBoids\n"
    printf "%d\n" "$boidCount"
    printf "trialNum\t time\n"

    trialNum=1
    while [[ $trialNum -le $num_trials ]]
    do
        printf "%d\t" "$trialNum"
        c="$bin 1 $boidCount"
        # printf "$c\t"
        $c
        ((trialNum++))
        printf "\n"
    done

    while [[ $trialNum -le $max_trials ]]
    do
        printf "%d\n" "$trialNum"
        ((trialNum++))
    done

    printf "\n\n\n"
done


