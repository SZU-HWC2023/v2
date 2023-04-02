#!/bin/sh
cd ..

./Robot ./v2/build/main -m maps/2.txt -d -f 2> ./v2/dev/err.txt

# ./Robot ./code/build/main -m maps/1.txt -f 2> ./code/dev/err.txt
# ./Robot ./code/build/main -m maps/2.txt -f 2> ./code/dev/err.txt
# ./Robot ./code/build/main -m maps/3.txt -f 2> ./code/dev/err.txt
# ./Robot ./code/build/main -m maps/4.txt -f 2> ./code/dev/err.txt

cd v2