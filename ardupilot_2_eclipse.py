# -*- coding: utf-8 -*-

import eclipse_cdt
import arduino
import file_utils

import os
import subprocess

from contextlib import contextmanager

@contextmanager
def change_working_directory(path):
    """
    contexrmanager that chages the current working directory and returns to the original one when it's closed.
    @param path: diretory
    """
    current_dir = os.getcwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(current_dir)

class ardupilot_project:
    """
    represents an ardupilot project.
    uses make the build the project in order to obtain the "large cpp file" that the makefile creates out of the pde files
    """
    
    def __init__(self,
                 ardupilot_path,
                 project_name,
                 tmp_dir,
                 make_arguments
                 ):
        """        
        @param ardupilot_path: path to ardupilot directory
        @param project_name: name of the project / sketch (e.g. "ArduCopter")
        @param tmp_dir: temp directory that is used by the makefile in $ardupilot_path$/$project_name$
        @param make_arguments: list of arguments that are passed to the make command 
        """
        
        self._path              = ardupilot_path
        self._project_name      = project_name
        self._tmp_dir           = tmp_dir
        self._make_arguments    = make_arguments[:]

        self._libraries_path = os.path.join(self._path, "libraries")
        self._project_path   = os.path.join(self._path, self._project_name)
        self._big_cpp_name   = "%s.cpp" %self._project_name
        self._big_cpp_path   = os.path.join(self._tmp_dir, "%s.build" %self._project_name, self._big_cpp_name)

        self._make()

        self._big_cpp_file       = arduino.big_cpp_file( os.path.join(self._tmp_dir,
                                                                  "%s.build" %self._project_name,
                                                                  self._big_cpp_name)
                                                    )
        self._needed_system_headers = self._big_cpp_file.get_includes()[1]
        self._ardupilot_lib         = arduino.arduino_lib(self._libraries_path)
            
    def _make(self, clean = False):
        """
        builds the project using make.
        @param clean: runs "make clean" beforehand.
        """
        with change_working_directory( self._project_path ):
            if clean:
                subprocess.check_call(["make", "clean"])
            subprocess.check_call(["make"] + self._make_arguments)

class eclipse_cdt_project:
    """
    stores some information about an eclipse cdt project.
    """
    
    def __init__(self, workspace_path, project_name):
        """
        @param workspace_path: path to an eclispe workspace
        @param project_name: name of an project contained in the workspace
        """
        self._workspace_path = workspace_path
        self._project_name   = project_name
        self._project_path   = os.path.join( self._workspace_path, self._project_name )
        
        self._cproject       = eclipse_cdt.cproject_xml( os.path.join(self._project_path, ".cproject") )

class ardupilot_eclipse:
    """
    class to convert an ardupilot project to an eclipse-project
    """
    def __init__(self,
                 eclipse_workspace_path,
                 eclipse_project_name,
                 ardupilot_path,
                 ardupilot_project_name,
                 ardupilot_tmp_dir,
                 ardupilot_make_arguments
                 ):
        """
        @param eclipse_workspace_path: path to an eclispe workspace
        @param eclipse_project_name: name of an eclipse project contained in the workspace
        @param ardupilot_path: path to ardupilot directory
        @param ardupilot_project_name: name of the project / sketch (e.g. "ArduCopter")
        @param ardupilot_tmp_dir: temp directory that is used by the makefile in $ardupilot_path$/$ardupilot_project_name$
        @param ardupilot_make_arguments: list of arguments that are passed to the make command 
        """
        self._ardupilot_project = ardupilot_project(ardupilot_path, ardupilot_project_name, ardupilot_tmp_dir, ardupilot_make_arguments)
        self._eclipse_project = eclipse_cdt_project(eclipse_workspace_path, eclipse_project_name)
        

    def set_includes(self, configuration):
        """
        sets all needed includes plaths in the eclipse project's config file
        @param configuration: configuration the includes are set to (e.g. "Release", "Debug", ...)
        """
        includes = self._ardupilot_project._ardupilot_lib.create_eclipse_include_paths(self._eclipse_project._project_name,
                                                                                       self._ardupilot_project._needed_system_headers,
                                                                                       "libraries")
        for tool in ["AVR Assembler",
                     "AVR Compiler",
                     "AVR C++ Compiler"]:
            self._eclipse_project._cproject.get_cconfiguration(configuration)\
                                           .get_toolChain("AVR-GCC Toolchain")\
                                           .get_tool(tool)\
                                           .set_include_paths( includes )

    def set_excludes(self, configuration):
        """
        sets all needed exclude plaths in the eclipse project's config file
        @param configuration: configuration the excludes are set to (e.g. "Release", "Debug", ...)
        """
        excludes = self._ardupilot_project._ardupilot_lib.create_eclipse_unused_libs_exclude(self._ardupilot_project._needed_system_headers,
                                                                                             "libraries",
                                                                                             ["examples", "tools"])
        self._eclipse_project._cproject.get_cconfiguration(configuration)\
                                       .set_excludes( excludes )

    def write_xml(self):
        """
        writes the modifications done to the eclipse config file back into the file.
        """
        self._eclipse_project._cproject.write_back()

    def copy_ardupilot_to_eclipse(self, link = False, dont_include_pde = False, ask = True):
        """
        copies / symlinks the files of the ardupilot-project into the eclipse project and generates a "main" cpp file.
        
        The "main" cpp is either the one generated by the makefile or one that uses include-statements instead of the pde files' content.
        
        @param link: True => symlink files/directories instead of copying them
        @param dont_include_pde: True => use the "big cpp file" instead of the one that includes the pde files
        @param ask: True => get a confirmation before copying / deleting / linking files
        """


        eclipse_project_path = self._eclipse_project._project_path
        eclipse_lib_path     = os.path.join(eclipse_project_path, "libraries")
        eclipse_ardu_path    = os.path.join(eclipse_project_path, self._ardupilot_project._project_name)
        eclipse_big_cpp_path = os.path.join(eclipse_ardu_path, self._ardupilot_project._big_cpp_name)
        
        ardupilot_libraries_path = self._ardupilot_project._libraries_path
        ardupilot_project_path   = self._ardupilot_project._project_path
        ardupilot_big_cpp_path   = self._ardupilot_project._big_cpp_path
    
        # delete libs
        if not file_utils.rm(eclipse_lib_path, ask): return
        
        # copy / link libs
        if link:
            if not file_utils.linkfile(ardupilot_libraries_path, eclipse_lib_path,       ask): return
        else:
            if not file_utils.copytree(ardupilot_libraries_path, eclipse_lib_path, None, ask): return

        # delete project
        if not file_utils.rm(eclipse_ardu_path, ask): return
        
        # copy / link project
        includes = ['*.c++', '*.cpp', '*.c', '*.h', '*.hpp']
        if not dont_include_pde: includes.append('*.pde')
        include = file_utils.copyonly(includes)
        if link:
            if not file_utils.linktree(ardupilot_project_path, eclipse_ardu_path, include, ask): return
        else:
            if not file_utils.copytree(ardupilot_project_path, eclipse_ardu_path, include, ask): return
            
        # copy / generate big cpp file
        if dont_include_pde:
            if not file_utils.copyfile(ardupilot_big_cpp_path, eclipse_big_cpp_path, ask): return
        else:
            if not file_utils.get_confirmation('genereate "%s"?' %eclipse_big_cpp_path, ask): return
            self._ardupilot_project._big_cpp_file.build_main_cpp(eclipse_big_cpp_path)
            # rename *.pde to *.pde.hpp
            for f in os.listdir(eclipse_ardu_path):
                if f.endswith(".pde"):
                    fp = os.path.join(eclipse_ardu_path, f)
                    os.rename(fp, fp + ".hpp")
