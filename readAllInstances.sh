#!/bin/bash
for filename in instances/*.txt; do
    echo "Running python graphgen.py -R $filename save"
    python graphgen.py -R $filename
done