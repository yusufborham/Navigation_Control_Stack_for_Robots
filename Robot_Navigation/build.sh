#!/bin/bash

# Check if at least the source file is provided
if [ $# -lt 1 ]; then
    echo "Usage: $0 <source_file.c> [output_binary]"
    exit 1
fi

file="$1"
# Default output name to the source filename without the .c extension if $2 is omitted
obj="${2:-${file%.*}}"

gcc -std=c11 -Wall -Wextra "$file" \
    TreeFoof/tree.c \
    TreePriorityQueue/treePriorityQueue.c \
    -o "$obj" -lm -O3 -march=native -flto