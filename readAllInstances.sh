#!/bin/bash
for filename in instances/*.txt; do
    echo "Running python motion.py -R $filename save"
    python motion.py -R $filename
done
