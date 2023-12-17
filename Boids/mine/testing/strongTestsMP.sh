#!/bin/bash

bin="../tsglBoidsMP"
printf "Start of strong tests MP %s " "$bin"; date; lscpu


# Read the user's input at start to set the number of trials
num_trials=1
if [ "$1" ]
then 
    num_trials="$1"
fi


max_trials=16
if [[ $num_trials -ge $max_trials ]]
then
    num_trials=$max_trials
fi


boidsCounts=(128 256 512 1024 2048 4096 8192)

threadsCounts=(1 2 3 4 6)

performLine () {
    printf "%d\t" "$trialNum"

    for threadNum in "${threadsCounts[@]}"
    do
        c="$bin $threadNum $boidCount"
        $c
        printf "\t"
    done
    
    printf "\n"
}


for boidCount in "${boidsCounts[@]}"
do
    printf "numBoids\t numThreads\n"
    printf "%d\t" "$boidCount"

    for threadNum in "${threadsCounts[@]}"
    do
        printf "%d\t" "$threadNum"
    done
    printf "\n"
    printf "trialNum\t time\n"

    trialNum=1
    while [ $trialNum -le $num_trials ]
    do
        performLine
        ((trialNum++))
    done

    while [ $trialNum -le $max_trials ]
    do
        printf "%d\n" "$trialNum"
        ((trialNum++))
    done
    printf "\n\n"

done




