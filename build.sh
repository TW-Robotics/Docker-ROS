#!/bin/bash

LABEL="melodic"

docker build  \
        --rm \
        --tag georgno/fhtw-ros:"$LABEL" \
        --file Dockerfile .