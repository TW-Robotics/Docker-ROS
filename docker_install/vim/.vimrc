set nocompatible              " be iMproved, required
filetype off                  " required

" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()
" alternatively, pass a path where Vundle should install plugins
"call vundle#begin('~/some/path/here')

" let Vundle manage Vundle, required
Plugin 'VundleVim/Vundle.vim'
Plugin 'Valloric/YouCompleteMe'
Plugin 'airblade/vim-gitgutter'
Plugin 'kana/vim-operator-user'
Plugin 'rhysd/vim-clang-format'
Plugin 'vim-airline/vim-airline'
Plugin 'vim-airline/vim-airline-themes'
Plugin 'altercation/vim-colors-solarized'
Plugin 'tpope/vim-fugitive'
Plugin 'SirVer/ultisnips'
Plugin 'tomasr/molokai'
"Plugin 'taketwo/vim-ros'

" All of your Plugins must be added before the following line
call vundle#end()            " required
set encoding=utf-8
let g:ycm_global_ycm_extra_conf = '~/.vim/bundle/.ycm_extra_conf.py'
let g:ycm_add_preview_to_completeopt = 1
let g:ycm_autoclose_preview_window_after_insertion = 1
let g:ycm_enable_diagnostic_signs = 1
let g:ycm_use_ultisnips_completer = 1
let g:clang_format#command = 'clang-format'
let g:clang_format#code_style = 'llvm'
let g:clang_format#style_options ={
    \ "AccessModifierOffset" : -4,
    \ "AlignAfterOpenBracket": "Align",
    \ "AlignOperands": "true",
    \ "AlignTrailingComments": "true",
    \ "AlwaysBreakAfterReturnType": "None",
    \ "BinPackArguments": "false",
    \ "BinPackParameters": "false",
    \ "BreakBeforeBinaryOperators": "NonAssignment",
    \ "BreakBeforeBraces": "Attach",
    \ "BreakConstructorInitializersBeforeComma": "true",
    \ "CompactNamespaces": "true",
    \ "ColumnLimit": 120,
    \ "ConstructorInitializerAllOnOneLineOrOnePerLine": "true",
    \ "ConstructorInitializerIndentWidth": 8,
    \ "ContinuationIndentWidth": 4,
    \ "IndentCaseLabels": "false",
    \ "IndentWidth": 4,
    \ "Language": "Cpp",
    \ "NamespaceIndentation": "None",
    \ "PointerAlignment": "Left",
    \ "ReflowComments": "true",
    \ "SortIncludes": "false",
    \ "SpaceAfterCStyleCast": "true",
    \ "SpaceBeforeAssignmentOperators": "true",
    \ "SpaceBeforeParens": "Never",
    \ "SpaceInEmptyParentheses": "true",
    \ "SpacesInAngles": "true",
    \ "SpacesInCStyleCastParentheses": "true",
    \ "SpacesInParentheses": "true",
    \ "SpacesInSquareBrackets": "true",
    \ "Standard": "Cpp11",
    \ "UseTab": "Never"}
let g:UltiSnipsExpandTrigger="<leader><tab>"
let g:UltiSnipsJumpForwardTrigger="<c-b>"
let g:UltiSnipsJumpBackwardTrigger="<c-z>"
" map to <Leader>cf in C++ code
autocmd FileType c,cpp,cc,objc,h,hpp nnoremap <buffer><Leader>cc :<C-u>ClangFormat<CR>
autocmd FileType c,cpp,cc,objc,h,hpp vnoremap <buffer><Leader>cc :ClangFormat<CR>
" if you install vim-operator-user
autocmd FileType c,cpp,cc,objc,h,hpp map <buffer><Leader>x <Plug>(operator-clang-format)
" Toggle auto formatting:
nmap <Leader>C :ClangFormatAutoToggle<CR>
"Everything after Vundle
filetype plugin indent on
autocmd BufRead,BufNewFile *.launch setfiletype xml
"autocmd BufRead,BufNewFile *.launch setfiletype roslaunch
au FileType tex setl sw=2 sts=2 et
set spelllang=en_us,de_de
autocmd FileType tex,markdown,text set spell
autocmd BufRead,BufNewFile COMMIT_EDITMSG setlocal spell
set grepprg=grep\ -nH\ $*
set tabstop=4
set shiftwidth=4
set expandtab
set foldmethod=indent
set foldlevelstart=2
set number
"set wildmenu
set wildmode=list:longest,full
set lazyredraw
let g:bufferline_echo = 0
set noshowcmd
set showmatch
let g:tex_flavor='latex'
let g:Tex_DefaultTargetFormat='pdf'
let g:Tex_MultipleCompileFormats='pdf,dvi'
let g:airline#extensions#tabline#enabled = 1
let g:airline#extensions#ycm#enabled = 1
let g:airline_skip_empty_sections = 1
let g:airline_powerline_fonts=0
set laststatus=2
set t_Co=256
let g:solarized_termcolors=256
try
    colorscheme solarized
catch /^Vim\%((\a\+)\)\=:E185/
    "echom "Solarized is not yet available"
endtry
if has('gui_running')
    set background=light
else
    set background=dark
endif
"nnoremap <F10> :set background=dark <CR>
"nnoremap <F11> :set background=light <CR>
nnoremap <F9> :let &background = ( &background == "dark"? "light" : "dark" )<CR>
"if !exists('g:airline_symbols')
"  let g:airline_symbols = {}
"endif
let g:airline_powerline_fonts = 1

"let g:airline_left_sep = ''
"let g:airline_left_alt_sep = ''
"let g:airline_right_sep = ''
"let g:airline_right_alt_sep = ''
"let g:airline_symbols.space = "\ua0"
"let g:airline_symbols.linenr = "␤"
"let g:airline_symbols.branch = "⎇ "
"let g:airline_symbols.readonly = ''

inoremap <c-x><c-k> <c-x><c-k>
hi CursorLine   cterm=NONE ctermbg=darkred ctermfg=white guibg=darkred guifg=white
hi CursorColumn cterm=NONE ctermbg=darkred ctermfg=white guibg=darkred guifg=white
nnoremap <F4> :set cursorline! cursorcolumn!<CR>
nnoremap <F5> :set cursorline! <CR>
nnoremap <F6> :set cursorcolumn!<CR>
nnoremap <F2> :set invpaste paste?<CR>
set pastetoggle=<F2>
" Splits
nnoremap <C-J> <C-W><C-J>
nnoremap <C-K> <C-W><C-K>
nnoremap <C-L> <C-W><C-L>
nnoremap <C-H> <C-W><C-H>
set splitbelow
set splitright
set noshowmode
"inoremap <Esc> <Esc>`^
