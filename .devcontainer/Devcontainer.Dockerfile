FROM ghcr.io/tw-robotics/docker-ros:latest

RUN apt-get update && apt-get install -y dos2unix psmisc --no-install-recommends && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
