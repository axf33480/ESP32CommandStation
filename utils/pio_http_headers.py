#######################################################################
# ESP32 COMMAND STATION
#
# COPYRIGHT (c) 2017-2019 Mike Dunston
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses
#######################################################################

Import("env")

from resource_embedder import process_all
from os.path import join

def generate_embedded_resources(source, target, env):
  process_all(join(env.subst('$PROJECT_DIR'), 'data'),
              join(env.subst('$PROJECT_DIR'), 'include', 'generated'))

env.AddPreAction('$BUILD_DIR/src/Interfaces/WebServer.cpp.o', generate_embedded_resources)
