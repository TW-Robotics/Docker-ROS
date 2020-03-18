# This file is NOT licensed under the GPLv3, which is the license for the rest
# of YouCompleteMe.
#
# Here's the license text for this file:
#
# This is free and unencumbered software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.
#
# In jurisdictions that recognize copyright laws, the author or authors
# of this software dedicate any and all copyright interest in the
# software to the public domain. We make this dedication for the benefit
# of the public at large and to the detriment of our heirs and
# successors. We intend this dedication to be an overt act of
# relinquishment in perpetuity of all present and future rights to this
# software under copyright law.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# For more information, please refer to <http://unlicense.org/>

import os
import ycm_core

def getRosIncludePaths():
    import os
    try:
        from rospkg import RosPack
    except ImportError:
        return []
    rospack = RosPack()
    includes = []
    cDirectory = os.getcwd()
    if "firmware" in cDirectory:
        for p in os.path.expandvars('$ROS_PACKAGE_PATH').split(':'):
            if os.path.exists( p + '/../build/pas_firmware/ros_lib'):
                includes.append( p + '/../build/pas_firmware/ros_lib')
        for p in rospack.list():
            if os.path.exists(rospack.get_path(p) + '/firmware/include'):
                includes.append(rospack.get_path(p) + '/firmware/include')
    else:
        for p in os.path.expandvars('$ROS_PACKAGE_PATH').split(':'):
            if os.path.exists( p + '/../devel/include'):
                includes.append( p + '/../devel/include')
        for p in rospack.list():
            if os.path.exists(rospack.get_path(p) + '/include'):
                includes.append(rospack.get_path(p) + '/include')
        distros = ['melodic', 'lunar', 'kinetic', 'jade', 'indigo', 'hydro']
        for distro in distros:
            if os.path.exists('/opt/ros/'+distro+'/include'):
                    includes.append('/opt/ros/'+distro+'/include')
    print('ROS Paths: ' + ' '.join(includes))
    return includes

def getAVRPaths():
    import os
    includes = []
    if os.path.exists( '/usr/share/arduino/hardware/arduino/cores/arduino' ):
        includes.append( '/usr/share/arduino/hardware/arduino/cores/arduino' )
    if os.path.exists( '/usr/share/arduino/hardware/tools/avr/lib/avr/include' ):
        includes.append( '/usr/share/arduino/hardware/tools/avr/lib/avr/include' )
    return includes

def getRosIncludeFlags():
    import os
    cDirectory = os.getcwd()
    includes = getRosIncludePaths()
    flags = []
    if "firmware" in cDirectory:
        for include in includes:
            flags.append('-I'+include)
            flags.append('-DARDUINO=100')
            flags.append('-D__AVR_ATmega328P__')
            AVRIncludes = getAVRPaths()
            for avr in AVRIncludes:
                flags.append( '-isystem'+avr )
    else:
        for include in includes:
            flags.append('-isystem'+include)
    print('ROS dev path: ' + ' '.join(flags))
    return flags

def getFHTWDevPath():
    import os
    flags = []
    cwd = os.getcwd()
    #if os.path.exists( cwd + '/include' ):
    #    flags.append( '-I' + cwd + '/include' )
    #if os.path.exists( cwd + '/../include' ):
    #    flags.append( '-I' + cwd + '/../include' )
    #if os.path.exists( cwd + '/../../include' ):
    #    flags.append( '-I' + cwd + '/../../include' )
    dirs = []
    if cwd.endswith("/build"):
        cwd = cwd[:-6]
    for root, directories, filenames in os.walk(cwd, topdown=True):
        if ".git" in root or "include" in root:
            continue
        for dName in directories:
            if ".git" in dName:
                continue;
            if dName == "include" or dName == "messmer_gitversion" or dName=="src_generated":
                dirs.append(root+"/"+dName)
        #else:
        #    if not root.endswith("/include"):
        #        if filter(lambda x: x.endswith(".h") or x.endswith(".hpp"),filenames):
        #            dirs.append(root)
    for dirName in dirs:
        if "thirdparty" in dirName or "bin" in dirName:
            flags.append('-isystem'+dirName)
        else:
            flags.append('-I'+dirName)
    flags.sort()
    print('Local dev path: ' + ' '.join(flags))
    return flags

def getSTDLibraries():
    import os
    flags = []
    path = '/usr/include/c++'
    if os.path.exists( path ):
        subdirs = next(os.walk( path ))
        subdirs[1].sort(reverse=True)
        flags.append('-isystem'+subdirs[0]+'/'+subdirs[1][0])
        x86Include='/usr/include/x86_64-linux-gnu/c++/'+subdirs[1][0]
        if os.path.exists( x86Include):
            flags.append('-isystem'+x86Include)
    flags.append('-isystem/usr/include')
    flags.append('-isystem/usr/local/include')
    print('Libraries: ' + ' '.join(flags))
    return flags

def getCompileOptions():
    flags = [
                '-Wall',
                '-Wextra',
                '-pedantic',
                '-Wfloat-equal',
                '-Wold-style-cast'
                '-x','c++',
                '-std=c++17'
            ]
    print('Compile-flags: ' + ' '.join(flags))
    return flags

# These are the compilation flags that will be used in case there's no
# compilation database set (by default, one is not set).
# CHANGE THIS LIST OF FLAGS. YES, THIS IS THE DROID YOU HAVE BEEN LOOKING FOR.
flags = getCompileOptions() + getSTDLibraries() + getFHTWDevPath() + getRosIncludeFlags()


# Set this to the absolute path to the folder (NOT the file!) containing the
# compile_commands.json file to use that instead of 'flags'. See here for
# more details: http://clang.llvm.org/docs/JSONCompilationDatabase.html
#
# You can get CMake to generate this file for you by adding:
#   set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )
# to your CMakeLists.txt file.
#
# Most projects will NOT need to set this to anything; you can just change the
# 'flags' list of compilation flags. Notice that YCM itself uses that approach.
compilation_database_folder = ''

if os.path.exists( compilation_database_folder ):
  database = ycm_core.CompilationDatabase( compilation_database_folder )
else:
  database = None

SOURCE_EXTENSIONS = [ '.cpp', '.cxx', '.cc', '.c', '.m', '.mm' ]

def DirectoryOfThisScript():
  return os.path.dirname( os.path.abspath( __file__ ) )


def MakeRelativePathsInFlagsAbsolute( flags, working_directory ):
  if not working_directory:
    return list( flags )
  new_flags = []
  make_next_absolute = False
  path_flags = [ '-isystem', '-I', '-iquote', '--sysroot=' ]
  for flag in flags:
    new_flag = flag

    if make_next_absolute:
      make_next_absolute = False
      if not flag.startswith( '/' ):
        new_flag = os.path.join( working_directory, flag )

    for path_flag in path_flags:
      if flag == path_flag:
        make_next_absolute = True
        break

      if flag.startswith( path_flag ):
        path = flag[ len( path_flag ): ]
        new_flag = path_flag + os.path.join( working_directory, path )
        break

    if new_flag:
      new_flags.append( new_flag )
  return new_flags


def IsHeaderFile( filename ):
  extension = os.path.splitext( filename )[ 1 ]
  return extension in [ '.h', '.hxx', '.hpp', '.hh' ]


def GetCompilationInfoForFile( filename ):
  # The compilation_commands.json file generated by CMake does not have entries
  # for header files. So we do our best by asking the db for flags for a
  # corresponding source file, if any. If one exists, the flags for that file
  # should be good enough.
  if IsHeaderFile( filename ):
    basename = os.path.splitext( filename )[ 0 ]
    for extension in SOURCE_EXTENSIONS:
      replacement_file = basename + extension
      if os.path.exists( replacement_file ):
        compilation_info = database.GetCompilationInfoForFile(
          replacement_file )
        if compilation_info.compiler_flags_:
          return compilation_info
    return None
  return database.GetCompilationInfoForFile( filename )


def FlagsForFile( filename, **kwargs ):
  if database:
    # Bear in mind that compilation_info.compiler_flags_ does NOT return a
    # python list, but a "list-like" StringVec object
    compilation_info = GetCompilationInfoForFile( filename )
    if not compilation_info:
      return None

    final_flags = MakeRelativePathsInFlagsAbsolute(
      compilation_info.compiler_flags_,
      compilation_info.compiler_working_dir_ )

    ## NOTE: This is just for YouCompleteMe; it's highly likely that your project
    ## does NOT need to remove the stdlib flag. DO NOT USE THIS IN YOUR
    ## ycm_extra_conf IF YOU'RE NOT 100% SURE YOU NEED IT.
    #try:
    #  final_flags.remove( '-stdlib=libc++' )
    #except ValueError:
    #  pass
  else:
    relative_to = DirectoryOfThisScript()
    final_flags = MakeRelativePathsInFlagsAbsolute( flags, relative_to )

  return {
    'flags': final_flags,
    'do_cache': True
  }
