# FractalImages
# Copyright (C) 2012  Sven Hertle
# 
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
# 
# This program is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE. See the GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>. 

# Find TinyXML2
#
# TinyXML2_FOUND		True if TinyXML2 was found
# TinyXML2_INCLUDE_DIR		Directory with headers
# TinyXML2_LIBRARIES		List of libraries
#

find_path(TinyXML2_INCLUDE_DIR "tinyxml2.h")

find_library(TinyXML2_LIBRARIES NAMES "tinyxml2")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args("TinyXML2" DEFAULT_MSG TinyXML2_INCLUDE_DIR TinyXML2_LIBRARIES)
set(TinyXML2_FOUND ${TINYXML2_FOUND} CACHE BOOL "TinyXML2 was found or not" FORCE)

mark_as_advanced(TinyXML2_INCLUDE_DIR TinyXML2_LIBRARIES)

message(STATUS "TinyXML2_INCLUDE_DIR: ${TinyXML2_INCLUDE_DIR}")
message(STATUS "TinyXML2_LIBRARIES: ${TinyXML2_LIBRARIES}")
