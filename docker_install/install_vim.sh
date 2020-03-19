#!/bin/bash
set -x
USERNAME="$1"
if ! [ -e "/home/$USERNAME/.vimrc" ]; then
  echo "Installing vim config"
  vimconfig="$(find / -type f -name '.vimrc')"
  if [ -n "$vimconfig" ]; then
    cp "$vimconfig" "/home/$USERNAME/"
  fi
  if ! [ -e "/home/$USERNAME/.vim/bundle/Vundle.vim" ]; then
    mkdir -p "/home/$USERNAME/.vim/bundle"
    ycmdconfig="$(find / -type f -name '.ycm_extra_conf.py')"
    if [ -n "$ycmdconfig" ]; then
      cp "$ycmdconfig" "/home/$USERNAME/.vim/bundle/"
    fi
    (
      cd "/home/$USERNAME/.vim/bundle"
      git clone https://github.com/VundleVim/Vundle.vim
    )
    echo "Install all required Plugins"
    vim -c 'PluginInstall' -c 'q' -c 'q'
    if [ -d "/home/$USERNAME/.vim/bundle/YouCompleteMe" ]; then
        echo "Initialize YouCompleteMe"
        (
          cd "/home/$USERNAME/.vim/bundle/YouCompleteMe"
          python3 install.py --clang-completer
        )
    fi
  fi
fi
if ! [ -e "/home/$USERNAME/.tmux.conf" ]; then
  tmuxconfig="$(find / -type f -name '.tmux.conf')"
  if [ -n "$tmuxconfig" ]; then
    cp "$tmuxconfig" "/home/$USERNAME/"
  fi
fi
