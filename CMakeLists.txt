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


SET(PROJECTNAME taskPlayer)
PROJECT(${PROJECTNAME})

FILE(GLOB_RECURSE folder_header *.h)
FILE(GLOB_RECURSE folder_source *.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${iCubDev_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS}
		    		${ctrlLib_INCLUDE_DIRS}
		    		${YARP_INCLUDE_DIRS}}
                    ${modHelp_INCLUDE_DIRS}
                    ${objects_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
				     icubmod
				     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

