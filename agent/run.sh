#!/bin/bash

challenge="1"
host="localhost"
robname="robG11"
pos="0"
outfile="mapping.out"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call C4 for challenge 1
        python3 C1/pyRobot/mainRob.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call C4 for challenge 2
        python3 C2/mainRob.py -h "$host" -p "$pos" -r "$robname"
        ;;
    3)
        # how to call C4 for challenge 3
        echo "Challenge 3 was not implemented."
        ;;
esac

