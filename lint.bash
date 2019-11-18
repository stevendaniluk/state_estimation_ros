#!/usr/bin/env bash

# All the folders we want to lint
folders=(
    include/state_estimation_ros
    src
    test
)

# Get the root directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

for folder in "${folders[@]}"; do
    for filename in $DIR/$folder/*.{h,hpp,cpp}; do
        if [ -f "$filename" ]; then
            echo "Formatting "$filename
            clang-format -i $filename
        fi
    done
done
