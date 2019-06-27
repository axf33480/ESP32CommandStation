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

from resource_embedder import compress, embed;

def build_index_html_h(source, target, env):
    compress('%s/data/index.html' % env.subst('$PROJECT_DIR'),
             '%s/include/index_html.h' % env.subst('$PROJECT_DIR'),
             'indexHtmlGz')
    compress('%s/data/jquery.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_min_js.h' % env.subst('$PROJECT_DIR'),
             'jqueryJsGz')
    compress('%s/data/jquery.mobile-1.5.0-rc1.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_mobile_js.h' % env.subst('$PROJECT_DIR'),
             'jqueryMobileJsGz')
    compress('%s/data/jquery.mobile-1.5.0-rc1.min.css' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_mobile_css.h' % env.subst('$PROJECT_DIR'),
             'jqueryMobileCssGz')
    compress('%s/data/jquery.simple.websocket.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jquery_simple_websocket.h' % env.subst('$PROJECT_DIR'),
             'jquerySimpleWebSocketGz')
    compress('%s/data/jqClock-lite.min.js' % env.subst('$PROJECT_DIR'),
             '%s/include/jq_clock.h' % env.subst('$PROJECT_DIR'),
             'jqClockGz')
    embed('%s/data/ajax-loader.gif' % env.subst('$PROJECT_DIR'),
          '%s/include/ajax_loader.h' % env.subst('$PROJECT_DIR'),
          'ajaxLoader')

env.AddPreAction('$BUILD_DIR/src/Interfaces/WebServer.cpp.o', build_index_html_h)
