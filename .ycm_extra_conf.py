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
import re


# Set this to the absolute path to the folder (NOT the file!) containing the
# compile_commands.json file to use that instead of 'flags'. See here for
# more details: http://clang.llvm.org/docs/JSONCompilationDatabase.html
#
# Most projects will NOT need to set this to anything; you can just change the
# 'flags' list of compilation flags. Notice that YCM itself uses that approach.
compilation_database_folder = ''

# compile flags dict
# key flags:will add into the compiler cflags, its value is a comile_flags_dict or list or tuple or str
# key extension(<extensions list divie by |):  when the file extension in the list, the value will add into flags
# key command: first will execute the command, then split the out put by space and add them into flags
compile_flags_dict = {
'flags':[ #str, tuple, list, dict
    '-Wall',
    '-Wextra',
    #'-Werror',
    #'-Wc++98-compat',
    #'-Wno-long-long',
    '-Wno-variadic-macros',
    '-fexceptions',
    '-DNDEBUG',
    '-isystem',
    '/usr/include',
    '-isystem',
    '/usr/include/c++/4.8',
    '-isystem',
    '/usr/local/include',
    '-isystem',
    '/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/../include/c++/v1',
    '-isystem',
    '/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/../lib/clang/6.0/include',
    '-isystem',
    '/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/include'
    '-isystem',
    '/System/Library/Frameworks',
    '-isystem',
    '/Library/Frameworks',
],
'extension(.c|.cc)':[ #list tuple or dict
    '-std=gnu99',
    '-x',
    'c',
    ],
'extension(.cpp|.cxx|.h|.hpp|.hh|.hxx)':[
    '-std=gnu++11',
    '-x',
    'c++',
    '-isystem',
    '/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/../lib/c++/v1',
    '-I',
    'src',
    '-I',
    '/usr/include/mysql',
    '-I',
    '/usr/include/mysql++',
    ],
'command':#str tuple, list
    [
    #'python-config --cflags',
    ]

}

def compile_flags_parser(compile_item, fileextension):
    retlist = []
    def flagsFromCmd(cmds):
        cmdstr = []
        if isinstance(cmds, str):
            cmds = [cmds,]

        for cmd in cmds:
            f = os.popen(cmd, 'r')
            if f:
                for line in f.readlines():
                #if there is space in the path may cause some problem, but most file with no space
                    cmdstr.extend([ cmd if i == 0 else '-'+ cmd \
                        for i, cmd in enumerate(line.strip(' \n').split(' -'))])
                f.close()
        return cmdstr

    if isinstance(compile_item, str):
        retlist.append(compile_item)
    elif isinstance(compile_item, list) or isinstance(compile_item, tuple):
        for it in compile_item:
            retlist.extend( compile_flags_parser(it, fileextension))
    elif isinstance(compile_item, dict):
        flags = compile_item.get('flags', None)#make flags in the front
        if flags:
            retlist.extend(compile_flags_parser(flags, fileextension))
        for key, it in compile_item.iteritems():
            if key == 'flags':
                continue
            elif key.startswith('extension'):
                extensions = re.search('extensions?\((.*)\)',key)
                if extensions:
                    extensions = extensions.group(1).split('|')
                    if fileextension in extensions:
                        retlist.extend(compile_flags_parser(it, fileextension))
                else:
                    raise ValueError('extension key must be like "extension(.h|.c|.cpp)"')
            elif key == 'command':
                if isinstance(it, str) or isinstance(it, list) or isinstance(it, tuple):
                    retlist.extend(flagsFromCmd(it))
                else:
                    raise ValueError('command value must be tuple,str or list')
            else:
                raise ValueError('unsupport flag key, only support "flags, command, extension"')
    return retlist


def DirectoryOfThisScript():
  return os.path.dirname( os.path.abspath( __file__ ) )

if compilation_database_folder:
  if not compilation_database_folder.startswith('/'):
    compilation_database_folder = os.path.join( DirectoryOfThisScript(), compilation_database_folder )
  database = ycm_core.CompilationDatabase( compilation_database_folder )
else:
  database = None

SOURCE_EXTENSIONS = [ '.cpp', '.cxx', '.cc', '.c', '.m', '.mm' ]

def IsHeaderFile( filename ):
  extension = os.path.splitext( filename )[ 1 ]
  return extension in [ '.h', '.hxx', '.hpp', '.hh' ]

##########################################################
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

    # NOTE: This is just for YouCompleteMe; it's highly likely that your project
    # does NOT need to remove the stdlib flag. DO NOT USE THIS IN YOUR
    # ycm_extra_conf IF YOU'RE NOT 100% SURE YOU NEED IT.
	#try:
	#  final_flags.remove( '-stdlib=libc++' )
	#except ValueError:
	#  pass
  else:
    relative_to = DirectoryOfThisScript()
    extension = os.path.splitext( filename )[ 1 ]
    raw_flags = compile_flags_parser(compile_flags_dict, extension)
    final_flags = MakeRelativePathsInFlagsAbsolute( raw_flags, relative_to )

  return {
    'flags': final_flags,
    'do_cache': True
  }
