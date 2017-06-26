#!/bin/sh

build_stl() {
    echo "Generating STL file for [$1]"
    openscad $1  -D'$fn=64' -o stl_files/${1%.scad}.stl
    echo "STL [stl_files/${1%.scad}.stl] done!"
}

for FILE in *.scad; do
    build_stl $FILE &
done
