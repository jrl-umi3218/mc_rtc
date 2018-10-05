# Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of mc_rtc.
#
# mc_rtc is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# mc_rtc is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with mc_rtc.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function
try:
  from setuptools import setup
  from setuptools import Extension
except ImportError:
  from distutils.core import setup
  from distutils.extension import Extension

from Cython.Build import cythonize

import filecmp
import glob
import hashlib
import os
import shutil
import subprocess

from numpy import get_include as numpy_get_include

win32_build = os.name == 'nt'
debug_build = "$<CONFIGURATION>".lower() == "debug"

def exists_or_create(path):
  if not os.path.exists(path):
    os.makedirs(path)

def copy_if_different(f):
  in_ = '@CMAKE_CURRENT_SOURCE_DIR@/{}'.format(f)
  out_ = '{}/{}'.format(this_path, f)
  if not os.path.exists(out_) or not filecmp.cmp(in_, out_):
    shutil.copyfile(in_, out_)

packages = ['mc_control', 'mc_rbdyn', 'mc_rtc', 'mc_solver', 'mc_tasks']

this_path  = os.path.dirname(os.path.realpath(__file__))
for b in packages:
  exists_or_create(this_path + '/' + b)
  with open('{}/{}/__init__.py'.format(this_path, b), 'w') as fd:
    if b == 'mc_rtc':
      fd.write('import mc_control\n')
      fd.write('from . import gui\n')
    if b == 'mc_control':
      fd.write('from . import fsm\n')
    fd.write('from .{} import *\n'.format(b))

exists_or_create(this_path + '/mc_rtc/gui/')
with open('mc_rtc/gui/__init__.py', 'w') as fd:
  fd.write('from .gui import *\n')
exists_or_create(this_path + '/mc_control/fsm/')
with open('mc_control/fsm/__init__.py', 'w') as fd:
  fd.write('from .fsm import *\n')
exists_or_create(this_path + '/tests/')

glob_base = '@CMAKE_CURRENT_SOURCE_DIR@/'
bindings_src = [f for ext in ['pyx', 'pxd', 'pxi'] for f in glob.glob(glob_base + '*/*.' + ext) + glob.glob(glob_base + '*/*/*.' + ext) ]
bindings_src.extend(glob.glob(glob_base + 'tests/*.py'))
for f in bindings_src:
  copy_if_different(f.replace(glob_base, ''))

src_dir = '@CMAKE_CURRENT_SOURCE_DIR@/../../src/'
src_files = []
src_files += [ src_dir + f for f in '@mc_control_SRC@;@mc_rbdyn_SRC@;@mc_rtc_utils_SRC@;@mc_solver_SRC@;@mc_tasks_SRC@'.split(';') if len(f) ]
src_files += [ src_dir + f for f in '@mc_control_HDR@;@mc_rbdyn_HDR@;@mc_rtc_utils_HDR@;@mc_solver_HDR@;@mc_tasks_HDR@'.split(';') if len(f) ]
for b in packages:
  b_path = '{}/{}/'.format(this_path, b)
  suffixes = [ ('c_', '.pxd'), ('', '.pyx', ('', '.pxd'), '../include/', '.hpp') ]
  b_files = ['{}{}{}{}'.format(b_path, s[0], b, s[1]) for s in suffixes]
  b_files = [ f for f in b_files if os.path.exists(f) ]
  src_files += b_files

sha512 = hashlib.sha512()
for f in src_files:
  chunk = 2**16
  with open(f, 'r') as fd:
    while True:
      data = fd.read(chunk)
      if data:
        sha512.update(data.encode('ascii'))
      else:
        break
version_hash = sha512.hexdigest()[:7]

class pkg_config(object):
  def __init__(self):
    self.compile_args = []
    self.include_dirs = [ x for x in '$<TARGET_PROPERTY:mc_control,INCLUDE_DIRECTORIES>'.split(';') if len(x) ]
    self.include_dirs.append('@CMAKE_CURRENT_SOURCE_DIR@/include')
    self.library_dirs = [ x for x in '$<TARGET_PROPERTY:mc_control,LINK_FLAGS>'.split(';') if len(x) ]
    self.libraries = ['mc_control', 'mc_rbdyn', 'mc_rtc_utils', 'mc_solver', 'mc_tasks', 'mc_rtc_ros', 'mc_rtc_gui', 'mc_control_fsm']
    if debug_build:
      self.libraries = [ '{}{}'.format(l, '@PROJECT_DEBUG_POSTFIX@') for l in self.libraries ]
    self.libraries += [ os.path.splitext(os.path.basename(b))[0].replace('lib','') for b in '@Boost_LIBRARIES@'.split(';') if len(b) ]
    mc_rtc_location = '$<TARGET_FILE:mc_control>'
    mc_control_fsm_location = '$<TARGET_FILE:mc_control_fsm>'
    self.library_dirs.append(os.path.dirname(mc_rtc_location))
    self.library_dirs.append(os.path.dirname(mc_control_fsm_location))
    self.found = True

python_libs = []
python_lib_dirs = []
python_others = []

config = pkg_config()

config.compile_args.append('-std=c++11')
for o in python_others:
  config.compile_args.append(o)
config.include_dirs.append(os.getcwd() + "/include")
if not win32_build:
  config.library_dirs.extend(python_lib_dirs)
  config.libraries.extend(python_libs)
else:
  config.compile_args.append("-DWIN32")

def GenExtension(name, pkg, ):
  pyx_src = name.replace('.', '/')
  pyx_src = pyx_src + '.pyx'
  ext_src = pyx_src
  if pkg.found:
    return Extension(name, [ext_src], extra_compile_args = pkg.compile_args, include_dirs = pkg.include_dirs + [numpy_get_include()], library_dirs = pkg.library_dirs, libraries = pkg.libraries)
  else:
    print("Failed to find {}".format(pkg.name))
    return None

extensions = [ GenExtension('{}.{}'.format(p, p), config) for p in packages ]
extensions.append(GenExtension('mc_rtc.gui.gui', config))
extensions.append(GenExtension('mc_control.fsm.fsm', config))

extensions = [ x for x in extensions if x is not None ]

packages_data = {}
for p in packages:
  data = ['__init__.py']
  if os.path.exists('{}/{}/{}.pxd'.format(this_path, p, p)):
    data.append('{}.pxd'.format(p))
  if os.path.exists('{}/{}/c_{}.pxd'.format(this_path, p, p)):
    data.append('c_{}.pxd'.format(p))
  if p == 'mc_rtc':
    data += ['gui/__init__.py', 'gui/c_gui.pxd', 'gui/gui.pxd']
  if p == 'mc_control':
    data += ['fsm/__init__.py', 'fsm/c_fsm.pxd', 'fsm/fsm.pxd']
  packages_data[p] = data

extensions = cythonize(extensions)

setup(
    name = 'mc_rtc',
    version='1.0.0-{}'.format(version_hash),
    ext_modules = extensions,
    packages = packages,
    package_data = packages_data
)
