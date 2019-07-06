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

from resource_embedder import compress, embed
from os.path import join

def generate_embedded_resources(source, target, env):
    compress(join(env.subst('$PROJECT_DIR'), 'data', 'index.html'),
             join(env.subst('$PROJECT_DIR'), 'include', 'generated', 'index_html.h'),
             'indexHtmlGz')
    compress(join(env.subst('$PROJECT_DIR'), 'data', 'jquery.min.js'),
             join(env.subst('$PROJECT_DIR'), 'include', 'generated', 'jquery_min_js.h'),
             'jqueryJsGz')
    compress(join(env.subst('$PROJECT_DIR'), 'data', 'jquery.mobile-1.5.0-rc1.min.js'),
             join(env.subst('$PROJECT_DIR'), 'include', 'generated', 'jquery_mobile_js.h'),
             'jqueryMobileJsGz')
    compress(join(env.subst('$PROJECT_DIR'), 'data', 'jquery.mobile-1.5.0-rc1.min.css'),
             join(env.subst('$PROJECT_DIR'), 'include', 'generated', 'jquery_mobile_css.h'),
             'jqueryMobileCssGz')
    compress(join(env.subst('$PROJECT_DIR'), 'data', 'jquery.simple.websocket.min.js'),
             join(env.subst('$PROJECT_DIR'), 'include', 'generated', 'jquery_simple_websocket.h'),
             'jquerySimpleWebSocketGz')
    compress(join(env.subst('$PROJECT_DIR'), 'data', 'jqClock-lite.min.js'),
             join(env.subst('$PROJECT_DIR'), 'include', 'generated', 'jq_clock.h'),
             'jqClockGz')
    embed(join(env.subst('$PROJECT_DIR'), 'data', 'ajax-loader.gif'),
          join(env.subst('$PROJECT_DIR'), 'include', 'generated', 'ajax_loader.h'),
          'ajaxLoader')

env.AddPreAction('$BUILD_DIR/src/Interfaces/WebServer.cpp.o', generate_embedded_resources)
