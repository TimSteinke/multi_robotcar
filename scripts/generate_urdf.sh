#!/bin/bash

# This script generates an URDF from a template file with placeholder names and
# a robot name. The URDF is written to STDOUT, so that it can be directly used
# in launch files without storing on disk (see launch file entry
# <param  command=... /> )

if [[ -z "$2" ]] ; then
        echo "Usage: $0 <template_file> <car_name>" 1>&2;
        exit 1;
fi

echo $(cat "$1" | sed "s/placeholder/$2/g")
