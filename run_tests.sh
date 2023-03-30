#!/bin/bash

# vary seed from 1 to 10
seeds=(1 2 3 4 5 6 7 8 9 10)

maps=("1.txt" "2.txt" "3.txt" "4.txt")
total_score=0
cd ".."
# for seed in "${seeds[@]}"; do
#   # echo "Seed: $seed"
#   total_score=0
#   for map in "${maps[@]}"; do
#     output=$(./Robot "./yyj/build/main" -m "maps/$map" -f -s $seed 2> "./yyj/dev/err$map")
#     score=$(echo $output)
#     total_score=$((total_score + score))
#     # echo "Map $map score: $score"
#   done
#   echo "Seed: $seed Total score: $total_score"
# done
for map in "${maps[@]}"; do
  output=$(./Robot "./v2/build/main" -m "maps/$map" -f -s 115 2> "./v2/dev/err$map")
  score=$(echo $output | jq -r '.score')
  total_score=$((total_score + score))
  echo "Map $map score: $score"
done
echo "Total score: $total_score"
cd "v2"
