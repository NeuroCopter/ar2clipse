if __name__ == "__main__":
    """
    template main with hardcodes paths to automate the conversion process with python.
    """

    import ardupilot_2_eclipse
    import tempfile

    ardupilot_path         = "/home/tobias/git/ardupilot"
    ardupilot_project_name = "ArduCopter"
    
    eclipse_workspace = "/home/tobias/workspace_cpp"
    eclipse_project   = "ArduCopter"
    
    eclipse_configuration = "Release"

    system_tmp_dir = tempfile.gettempdir()


    a = ardupilot_2_eclipse.ardupilot_eclipse(eclipse_workspace,
                                              eclipse_project,
                                              ardupilot_path,
                                              ardupilot_project_name,
                                              system_tmp_dir,
                                              ["apm2-quad"]
                                              )

    a.copy_ardupilot_to_eclipse(link = True,
                                dont_include_pde = False,
                                ask = True
                                )
    
    a.set_includes(eclipse_configuration)
    a.set_excludes(eclipse_configuration)
    
    a.write_xml()
