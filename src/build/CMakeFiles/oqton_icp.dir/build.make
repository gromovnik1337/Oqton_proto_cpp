# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/build"

# Include any dependencies generated for this target.
include CMakeFiles/oqton_icp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/oqton_icp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/oqton_icp.dir/flags.make

CMakeFiles/oqton_icp.dir/oqton_icp.cpp.o: CMakeFiles/oqton_icp.dir/flags.make
CMakeFiles/oqton_icp.dir/oqton_icp.cpp.o: ../oqton_icp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/oqton_icp.dir/oqton_icp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/oqton_icp.dir/oqton_icp.cpp.o -c "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/oqton_icp.cpp"

CMakeFiles/oqton_icp.dir/oqton_icp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/oqton_icp.dir/oqton_icp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/oqton_icp.cpp" > CMakeFiles/oqton_icp.dir/oqton_icp.cpp.i

CMakeFiles/oqton_icp.dir/oqton_icp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/oqton_icp.dir/oqton_icp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/oqton_icp.cpp" -o CMakeFiles/oqton_icp.dir/oqton_icp.cpp.s

# Object files for target oqton_icp
oqton_icp_OBJECTS = \
"CMakeFiles/oqton_icp.dir/oqton_icp.cpp.o"

# External object files for target oqton_icp
oqton_icp_EXTERNAL_OBJECTS =

oqton_icp: CMakeFiles/oqton_icp.dir/oqton_icp.cpp.o
oqton_icp: CMakeFiles/oqton_icp.dir/build.make
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_people.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libboost_system.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libboost_regex.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libqhull.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libfreetype.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libz.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libjpeg.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpng.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libtiff.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libexpat.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_features.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_search.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_io.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libpcl_common.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libfreetype.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
oqton_icp: /usr/lib/x86_64-linux-gnu/libz.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libGLEW.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libSM.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libICE.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libX11.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libXext.so
oqton_icp: /usr/lib/x86_64-linux-gnu/libXt.so
oqton_icp: CMakeFiles/oqton_icp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable oqton_icp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/oqton_icp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/oqton_icp.dir/build: oqton_icp

.PHONY : CMakeFiles/oqton_icp.dir/build

CMakeFiles/oqton_icp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/oqton_icp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/oqton_icp.dir/clean

CMakeFiles/oqton_icp.dir/depend:
	cd "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src" "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src" "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/build" "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/build" "/media/gromovnik/Vice_SSD/02. Documents/CV/Applications/Oqton/Learning/Oqton_proto_cpp/src/build/CMakeFiles/oqton_icp.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/oqton_icp.dir/depend

