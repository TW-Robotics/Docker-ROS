FROM georgno/fhtw-ros:latest

# Update and get rid of python2
RUN apt-get update && apt-get install -y --no-install-recommends python-is-python3
# Install Jupyterlab
RUN pip3 install --upgrade pip
RUN pip3 install jupyterlab
RUN pip3 install --upgrade jupyter_core jupyter_client
RUN pip3 install matplotlib numpy

# Set shell env variable for jupyterlab (this fixes autocompletion in web-based shell)
ENV SHELL=/bin/bash 

# Remove TMUX Autostart, since we don't need it in JupyterLab
RUN sed -i '$ d' ~/.bashrc
RUN sed -i '$ d' ~/.bashrc
