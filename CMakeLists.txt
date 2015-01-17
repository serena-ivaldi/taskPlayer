# 
# Copyright (C) 2015 CODYCO Project
# Author: Serena Ivaldi <serena.ivaldi@inria.fr>
# 
# Copyright (C) 2011-2012 MACSi Project - www.macsi.isir.upmc.fr
# Author: Serena Ivaldi <serena.ivaldi@isir.upmc.fr>
#
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details
#


cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME taskPlayer)
PROJECT(${PROJECTNAME})

# updated list of commands due to decoupling from the macsi cmakelist
find_package(YARP)
find_package(ICUB)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)
include(iCubHelpers)
#end

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${iCubDev_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
		    ${ctrlLib_INCLUDE_DIRS}
                    #${modHelp_INCLUDE_DIRS}
                    #${objects_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
				     icubmod
				     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

