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

import hashlib
import os
import subprocess

from numpy import get_include as numpy_get_include

win32_build = os.name == 'nt'

packages = ['mc_control', 'mc_rbdyn', 'mc_rtc', 'mc_solver', 'mc_tasks']

this_path  = os.path.dirname(os.path.realpath(__file__))
for b in packages:
  with open('{}/{}/__init__.py'.format(this_path, b), 'w') as fd:
    fd.write('from .{} import *\n'.format(b))

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
    self.include_dirs = [ x for x in '@MC_RTC_INCLUDE_DIRECTORIES@'.split(';') if len(x) ]
    self.library_dirs = [ x for x in '@MC_RTC_LINK_FLAGS@'.split(';') if len(x) ]
    self.libraries = ['mc_control', 'mc_rbdyn', 'mc_rtc_utils', 'mc_solver', 'mc_tasks', 'mc_rtc_ros']
    self.libraries += [ os.path.splitext(os.path.basename(b))[0].replace('lib','') for b in '@Boost_LIBRARIES@'.split(';') if len(b) ]
    mc_rtc_location = '@MC_RTC_LOCATION@'
    self.library_dirs.append(os.path.dirname(mc_rtc_location))
    self.found = True

python_libs = []
python_lib_dirs = []
python_others = []
if not win32_build:
  tokens = subprocess.check_output(['python-config', '--ldflags']).split()
  tokens = [ token.decode('ascii') for token in tokens ]
  for token in tokens:
    flag = token[:2]
    value = token[2:]
    if flag == '-l':
      python_libs.append(value)
    elif flag == '-L':
      python_lib_dirs.append(value)
    elif token[:1] == '-':
      python_others.append(token)

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
  cpp_src = pyx_src + '.cpp'
  pyx_src = pyx_src + '.pyx'
  ext_src = pyx_src
  if pkg.found:
    return Extension(name, [ext_src], extra_compile_args = pkg.compile_args, include_dirs = pkg.include_dirs + [numpy_get_include()], library_dirs = pkg.library_dirs, libraries = pkg.libraries)
  else:
    print("Failed to find {}".format(pkg.name))
    return None

extensions = [ GenExtension('{}.{}'.format(p, p), config) for p in packages ]

extensions = [ x for x in extensions if x is not None ]

packages_data = {}
for p in packages:
  data = ['__init__.py']
  if os.path.exists('{}/{}/{}.pxd'.format(this_path, p, p)):
    data.append('{}.pxd'.format(p))
  if os.path.exists('{}/{}/c_{}.pxd'.format(this_path, p, p)):
    data.append('c_{}.pxd'.format(p))
  packages_data[p] = data

extensions = cythonize(extensions)

setup(
    name = 'mc_rtc',
    version='1.0.0-{}'.format(version_hash),
    ext_modules = extensions,
    packages = packages,
    package_data = packages_data
)
