#!/bin/bash
set -x
if ! [ -e "$HOME/.vimrc" ]; then
  echo "Installing vim config"
  vimconfig="$(find / -type f -name '.vimrc')"
  if [ -n "$vimconfig" ]; then
    cp "$vimconfig" "$HOME/"
  fi
  if ! [ -e "$HOME/.vim/bundle/Vundle.vim" ]; then
    mkdir -p "$HOME/.vim/bundle"
    ycmdconfig="$(find / -type f -name '.ycm_extra_conf.py')"
    if [ -n "$ycmdconfig" ]; then
      cp "$ycmdconfig" "$HOME/.vim/bundle/"
    fi
    (
      cd "$HOME/.vim/bundle"
      git clone https://github.com/VundleVim/Vundle.vim
    )
    echo "Install all required Plugins"
    vim -c 'PluginInstall' -c 'q' -c 'q'
    if [ -d "$HOME/.vim/bundle/YouCompleteMe" ]; then
        echo "Initialize YouCompleteMe"
        (
          cd "$HOME/.vim/bundle/YouCompleteMe"
          python3 install.py --clang-completer
        )
    fi
  fi
fi
if ! [ -e "$HOME/.tmux.conf" ]; then
  tmuxconfig="$(find / -type f -name '.tmux.conf')"
  if [ -n "$tmuxconfig" ]; then
    cp "$tmuxconfig" "$HOME/"
  fi
fi
