# -*- coding: utf-8 -*-

import xml.etree.ElementTree

class cproject_xml:
    """
    This class represents Eclipse-CDT's .cproject config-xml files.
    
    It provides methods to edit some common attributes like include-paths. 
    """

    def __init__(self, path):
        """
        @param path: path to the .cproject xml file
        @type path: string
        """
        self._path = path
        self._tree = xml.etree.ElementTree.parse(path)

    def write_back(self):
        """
        writes all changes made to this object back to the .cproject file.
        """
        def indent(node, level):
            ind = '\n' + level * '\t'
            # zeilenumbruch + einrückung für geschwisterknoten
            if node.tail is None or not len(node.tail.strip()):
                node.tail = ind
            # kinder bearbeiten:
            if len(node):
                # zeilenumbruch + tiefere einrückung fürs erste kind
                if node.text is None or not len(node.text.strip()):
                    node.text = ind + '\t'
                for child in node:
                    indent(child, level + 1)
                # beim letzten kind die zu tiefe einrückung für den nun
                #  folgenden geschwisterknoten von node korrigieren
                if child.tail is None or not child.tail.strip():
                    child.tail = ind
            # keine kinder --> text entfernen
            else:
                if node.text is None or not node.text.strip():
                    node.text = None                

        indent(self._tree.getroot(), 0)
        with open(self._path, 'w') as f:
            f.write('<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n<?fileVersion 4.0.0?>')
            self._tree.write(f, encoding = "UTF-8", xml_declaration = False)

    def get_ids(self):
        """
        some nodes in a .cproject file have "id"-attributes.
        This function returns an ascending duplicate-free list of these attributes.
        @rtype: list of strings
        """
        result = []
        for e in self._tree.getroot().iter():
            if "id" in e.attrib:
                tmp = e.attrib["id"]
                if tmp not in result:
                    result.append(tmp)
        result.sort()
        return result
    
    def get_id_nums(self):
        """
        some nodes in a .cproject file have "id"-attributes of which some end in an integer.
        This function returns an ascending duplicate-free list of these integers.
        @see: get_ids
        @rtype: list of integers
        """
        result = []
        for i in self.get_ids():
            try:
                tmp = int(i.rsplit('.', 1)[-1])
            except ValueError:
                pass
            else:
                if tmp not in result:
                    result.append(tmp)
        result.sort()
        return result

    def create_unique_id_num(self):
        """
        some nodes in a .cproject file have "id"-attributes of which some end in an integer.
        This function return an integer that differs from all of them.
        @rtype: integer
        """
        return self.get_id_nums()[-1] + 1

    def get_cconfigurations(self):
        """
        an eclipse-cdt project has several configurations (Debug, Release, ...)
        This function return a list of objects that represent the corresponding nodes in the xml file.
        (configuration = release, debug, ...)
        @rtype: list of cproject_xml.cconfiguration objects
        """
        root = self._tree.getroot()
        configurations = [cproject_xml.cconfiguration(c, self) for c in root.iterfind("storageModule/cconfiguration")]
        return configurations

    def get_cconfiguration(self, cname):
        """
        @see get_configurations
        @param cname: name of the configuration
        @type cname: string
        @rtype: cproject_xml.cconfiguration
        @return: returns an oject that represents the configuration named "cname"
        """
        for config in self.get_cconfigurations():
            if config.get_name() == cname:
                return config
        raise KeyError('no cconfiguration named "%s" was found' %cname)

    class cconfiguration:
        """
        an eclipse-cdt project has several configurations (Debug, Release, ...)
        This class represents the corresponding nodes in the xml file.
        """

        def __init__(self, cconfiguration_node, parent_cproject_xml):
            """
            @param cconfiguration_node: node in the xml file that represents a configurations
            @type cconfiguration_node: xml.etree.ElementTree.Element
            @type parent_cproject_xml: cproject_xml
            """
            if cconfiguration_node.tag != "cconfiguration":
                raise ValueError
            self._cconfiguration_node = cconfiguration_node
            self._parent_cproject_xml = parent_cproject_xml

        def get_name(self):
            """
            @rtype: string
            @return: name of this configuration (e.g. "Debug", "Release", ...)
            """
            for storage_module in self._cconfiguration_node.iterfind("storageModule"):
                if storage_module.get("moduleId", None) == "org.eclipse.cdt.core.settings":
                    return storage_module.attrib["name"]
            raise ValueError("cconfiguration node doesn't have a settings storageModule")


        def get_excluding_entry_node(self, create = False):
            """
            @param create: if the node isn't found, this options triggers it's creation
            @type create: bool 
            @rtype: xml.etree.ElementTree.Element
            @return: node that repesents the "exclude from build..." directories of this configuration / None if not present
            """
            configuration_node = self._cconfiguration_node.find("storageModule/configuration")
            sourceEntries_node = configuration_node.find("sourceEntries")
            if sourceEntries_node is None:
                if not create:
                    return None
                else:
                    sourceEntries_node = xml.etree.ElementTree.Element("sourceEntries", {})
                    configuration_node.append(sourceEntries_node)
            else:
                pass

            entry_node = sourceEntries_node.find("entry")
            if entry_node is None:
                if not create:
                    return None
                else:
                    entry_node = xml.etree.ElementTree.Element("entry", {"excluding" : "",
                                                                         "flags"     : "VALUE_WORKSPACE_PATH|RESOLVED",
                                                                         "kind"      : "sourcePath",
                                                                         "name"      : ""
                                                                         }
                                                               )
                    sourceEntries_node.append(entry_node)
            else:
                pass
            return entry_node

        def get_excludes(self):
            """
            @rtype: list of strings
            @return: list of directories that are exluded from this configuration's build ("exclude from build ...") 
            """
            entry_node = self.get_excluding_entry_node(False)
            if entry_node is None:
                return []
            else:
                exclude_string = entry_node.get("excluding")
                if len(exclude_string):
                    return exclude_string.split('|')
                else:
                    return []

        def append_excludes(self, exclude_list):
            """
            adds exludes-directories to the existing ones.
            @param exclude_list: directories that are excluded from this configuration's build ("exclude from build ...")
            @type exclude_list: list of strings
            """
            existing_excludes = self.get_excludes()
            new_excludes = []
            for ex in exclude_list:
                if ex not in existing_excludes and ex not in new_excludes:
                    new_excludes.append(ex)
            if len(new_excludes):
                entry_node = self.get_excluding_entry_node(True)
                entry_node.set("excluding", '|'.join(existing_excludes + new_excludes) )

        def clear_excludes(self):
            """
            clears the list of "exclude form build ..." directories
            """
            entry_node = self.get_excluding_entry_node(False)
            if entry_node is None:
                return
            else:
                entry_node.set("excluding", "")

        def set_excludes(self, exclude_list):
            """
            sets exludes-directories. (clears existings ones and sets these)
            @param exclude_list: directories that are excluded from this configuration's build ("exclude from build ...")
            @type exclude_list: list of strings
            """
            self.clear_excludes()
            self.append_excludes(exclude_list)

        def get_toolChains(self):
            """
            @return: list ob objects that represent the toolchains of a configuration (e.g. "AVR-GCC Toolchain", ...)
            @rtype: list of cproject_xml.cconfiguration.toolChain objects
            """
            root = self._cconfiguration_node
            toolchains = [cproject_xml.cconfiguration.toolChain(t, self) for t in root.iterfind("storageModule/configuration/folderInfo/toolChain")]
            return toolchains

        def get_toolChain(self, tname):
            """
            @return: object that represent the toolchain named "tname" of this configuration
            @rtype: cproject_xml.cconfiguration.toolChain object
            """
            for toolchain in self.get_toolChains():
                if toolchain.get_name() == tname:
                    return toolchain
            raise KeyError('no toolChain named "%s" was found' %tname)
            
        class toolChain:
            """
            This class represents a toolChain node in the c.project xml file.
            a toolChain node is a child of a cconfiguration node.
            toolChain e.g. "AVR-GCC Toolchain", ...
            """
            def __init__(self, toolChain_node, parent_cconfiguration):
                """
                @param toolChain_node: node in the xml file that represents a toolChain
                @type toolChain_node: xml.etree.ElementTree.Element
                @type parent_cconfiguration: cproject_xml.cconfiguration
                """
                if toolChain_node.tag != "toolChain":
                    raise ValueError
                self._toolChain_node = toolChain_node
                self._parent_cconfiguration = parent_cconfiguration

            def get_name(self):
                """
                @return: name of this toolChain (e.g. "AVR-GCC Toolchain", ...)
                @rtype: string
                """
                return self._toolChain_node.attrib["name"]
                
            def get_tools(self):
                """
                @return: list ob objects that represent the tools of a toolChain (e.g. "AVR Assembler", "GCC C++ Compiler", ...)
                @rtype: list of cproject_xml.cconfiguration.toolChain.tool objects
                """
                root = self._toolChain_node
                tools = [cproject_xml.cconfiguration.toolChain.tool(t, self) for t in root.iterfind("tool")]
                return tools

            def get_tool(self, tname):
                """
                @return: object that represent the tool named tname of a toolChain (e.g. "AVR Assembler", "GCC C++ Compiler", ...)
                @rtype: cproject_xml.cconfiguration.toolChain.tool object
                """
                for tool in self.get_tools():
                    if tool.get_name() == tname:
                        return tool
                raise KeyError('no tool named "%s" found' %tname)

            class tool:
                """
                This class represents a tool node in the c.project xml file.
                a tool node is a child of a toolChain node.
                tool e.g. "AVR Assembler", "GCC C++ Compiler", ...
                """
                def __init__(self, tool_node, parent_toolChain):
                    """
                    @param tool_node: node in the xml file that represents a tool
                    @type tool_node: xml.etree.ElementTree.Element
                    @type parent_toolChain: cproject_xml.cconfiguration.toolChain
                    """
                    if tool_node.tag != "tool":
                        raise ValueError
                    self._tool_node = tool_node
                    self._parent_toolChain = parent_toolChain
                
                
                """
                { TOOL_NAME : { OPTION_TYPE :  OPTION_ATTRIBS_DICT } }
                """
                _option_attributes =  {"AVR C++ Compiler" : {"includePath"  : {"name"       : "Include Paths (-I)",
                                                                               "superClass" : "de.innot.avreclipse.cppcompiler.option.incpath"
                                                                               }
                                                             },
                                       "AVR Assembler"    : {"includePath"  : {"name"       : "#include paths for preprocessor(-I)",
                                                                               "superClass" : "de.innot.avreclipse.asm.option.include.paths"
                                                                               }
                                                             },
                                       "AVR Compiler"     : {"includePath"  : {"name"       : "Include Paths (-I)",
                                                                               "superClass" : "de.innot.avreclipse.compiler.option.incpath"
                                                                               }
                                                             },
                                       "GCC C++ Compiler" : {"includePath"  : {"name"       : "Include paths (-I)",
                                                                               "superClass" : "gnu.cpp.compiler.option.include.paths"
                                                                               },
                                                             "includeFiles" : {"name"       : "Include files (-include)",
                                                                               "superClass" : "gnu.cpp.compiler.option.include.files"
                                                                               }
                                                             },
                                       "GCC C Compiler"   : {"includePath"  : {"name"       : "Include paths (-I)",
                                                                               "superClass" : "gnu.c.compiler.option.include.paths"
                                                                               },
                                                             "includeFiles" : {"name"       : "Include files (-include)",
                                                                               "superClass" : "gnu.c.compiler.option.include.files"
                                                                               }
                                                             }
                                       }

                def get_name(self):
                    """
                    @rtype: string
                    @return: name of this tool (e.g. "AVR Assembler", "GCC C++ Compiler", ...)
                    """
                    return self._tool_node.attrib["name"]

                def get_createable_options(self):
                    """
                    A tool supports several options (e.g. "includePath", "includeFiles") that are stored in child nodes.
                    If the node for a supported options isn't present, it needs to be created.                    
                    @return: options that can be created
                    @rtype: list of strings
                    """
                    if self.get_name() in self._option_attributes:
                        return self._option_attributes[self.get_name()].keys()
                    return []

                def option_is_createable(self, option_type):
                    """
                    @see: get_creatable_options
                    @param option_type: name of an option
                    @type option_type: string 
                    @rtype: bool
                    @return: true iff this tool has a template for creating the option named option_type
                    """
                    return option_type in self.get_createable_options()

                def get_option_node(self, option_type, create = False):
                    """
                    A tool has several options ("includePath", "includeFiles").
                    If the option isn't set, the coresponding node in the xml document doesn't exist.
                    Therefore it needs to be created, if the option is modified.
                    unfortunately their naming doesn't follow a common pattern. therefore this class provides a template
                    for creating the nodes. (the id attributes end in an integer. since it's meaning is unknown to the author,
                    the numer is coosen uniquely as a precaution)
                    
                    @param option_type: name of an option
                    @type option_type: string   
                    @param create: if the node isn't found, this options triggers it's creation
                    @type create: bool 
                    @raise KeyError: if create is True and the option isn't creatable,an exception is risen
                    @rtype: xml.etree.ElementTree.Element
                    @return: node that repesents the option name option_type of this tool / None if not present
                    """
                    for option in self._tool_node.iterfind("option"):
                        if option.get("valueType", None) == option_type:
                            return option
                    if not create:
                        return None
                    else:
                        if self.get_name() not in self._option_attributes:
                            raise KeyError('no option templates for tool "%s" specified' %( self.get_name() ) )

                        if not self.option_is_createable(option_type):
                            raise KeyError('no "%s" option template for tool "%s"' %( option_type, self.get_name() ) )

                        tool_option_attribs = self._option_attributes[self.get_name()][option_type]
                        
                        attrib = {"id"         : tool_option_attribs["superClass"] + ".%d" %self._parent_toolChain._parent_cconfiguration._parent_cproject_xml.create_unique_id_num(),
                                  "name"       : tool_option_attribs["name"],
                                  "superClass" : tool_option_attribs["superClass"],
                                  "valueType"  : option_type
                                  }
                        
                        new_option = xml.etree.ElementTree.Element("option", attrib)
                        self._tool_node.append( new_option )
                        return new_option                        

                def get_include_paths(self):
                    """
                    @return: list of paths that are passed to the compiler with the "-I" option
                    @rtype: list of strings
                    """
                    option_node = self.get_option_node("includePath", False)
                    if option_node is None:
                        return []
                    else:
                        return [value.attrib["value"] for value in option_node.findall("listOptionValue")]

                def append_include_paths(self, path_list):
                    """
                    appends the given paths to the list of inlcude paths.
                    @param path_list: list include plaths (that may contain eclipse-specific part e.g. "${workspace_loc:/PROJECT_X/libs/}")
                    @type path_list: list of strings
                    @raise KeyError: iff option node is non-existent and not createable
                    """
                    existing_includes = self.get_include_paths()
                    new_includes = []
                    for p in path_list:
                        if p not in existing_includes and p not in new_includes:
                            new_includes.append(p)
                    if len(new_includes):
                        option_node = self.get_option_node("includePath", True)
                        for p in new_includes:
                            option_node.append(xml.etree.ElementTree.Element("listOptionValue", {"builtIn" : "false", "value" : p}))

                def clear_include_paths(self):
                    """
                    clears the list of includes parths.
                    """
                    option_node = self.get_option_node("includePath", False)
                    if option_node is None:
                        return
                    else:
                        for value in option_node.findall("listOptionValue"):
                            option_node.remove(value)

                def set_include_paths(self, path_list):
                    """
                    sets the given paths as list of inlcude paths (= clear & append)
                    @param path_list: list include plaths (that may contain eclipse-specific part e.g. "${workspace_loc:/PROJECT_X/libs/}")
                    @type path_list: list of strings
                    @raise KeyError: iff option node is non-existent and not createable
                    """
                    self.clear_include_paths()
                    self.append_include_paths(path_list)

    def print_info(self, ind = 0):
        """
        most beautiful output of the parsed information you could imagine.
        @param ind: indention level of output
        @type ind: integer
        """
        inds = "  "
        for c in self.get_cconfigurations():
            print ind * inds + "CONFIG:", c.get_name()
            exs = c.get_excludes()
            if len(exs):
                ind += 1
                print ind * inds + "EXCLUDES:"
                ind += 1
                for ex in exs:
                    print ind * inds + ex
                ind -= 1
                ind -= 1
            ind += 1
            for tc in c.get_toolChains():
                print ind * inds + "TOOLCHAIN:", tc.get_name()
                ind += 1
                for t in tc.get_tools():
                    print ind * inds + "TOOL:", t.get_name()
                    co = t.get_createable_options()
                    if len(co):
                        ind += 1
                        print ind * inds + "CREATABLE OPTIONS:", co
                        ind -= 1
                    includes = t.get_include_paths()
                    if len(includes):
                        ind += 1
                        print ind * inds + "INCLUDE PATHS:"
                        ind += 1
                        for i in includes:
                            print ind * inds + i
                        ind -= 1
                        ind -= 1
                ind -= 1
            ind -= 1
        return
        
if __name__ == "__main__":
    """
    demonstrates usage of this module
    """
   
    avr_arducopter = "example_files/hidden_.cproject"

    paths = [avr_arducopter]

    def clear_all(xm_cproject):
        """
        clears include paths of all tools of all configurations
        """
        for c in xm_cproject.get_cconfigurations():
            c.clear_excludes()
            for tc in c.get_toolChains():
                for t in tc.get_tools():
                    t.clear_include_paths()
        return

    def add_if(xm_cproject):
        """
        adds sample strings to all include-paths / exclude-paths of all tools of all configurations (if they support this option)
        """
        for c in xm_cproject.get_cconfigurations():
            c.append_excludes(["<<<__NEW_EXCLUDE_PATH__>>>"])
            for tc in c.get_toolChains():
                for t in tc.get_tools():
                    if t.option_is_createable("includePath"):
                        t.append_include_paths(["<<<__NEW_INCLUDE_PATH__>>>"])
        return

    for p in paths:
        print "PATH:", p
        print "-------------------------------------------------"
        print "  READ :CPROJECT:"
        print "  ---------------"
        proj = cproject_xml(p)
        proj.print_info(2)
        print
        
        print "  ADD NEW PATHS:"
        print "  --------------"
        add_if(proj)
        proj.print_info(2)
        print

        print "  CLEAR ALL PATHS:"
        print "  ----------------"
        clear_all(proj)
        proj.print_info(2)
        print


# structure of the .cproject xml file
"""
<cproject ...>
    storageModule ...>
	<cconfiguration ...>
            <storageModule ...name="...">
            </storageModule>
            <storageModule moduleId="cdtBuildSystem" version="4.0.0">
                <configuration ...>
		    <folderInfo ...>
			<toolChain ...>
                            ...
                            <tool
                                id="de.innot.avreclipse.tool.assembler.winavr.base.792865251"
                                name="AVR Assembler"
                                superClass="de.innot.avreclipse.tool.assembler.winavr.base"
                            >
                                ...
                                <option
                                    id="de.innot.avreclipse.asm.option.include.paths.418778536"
                                    name="#include paths for preprocessor(-I)"
                                    superClass="de.innot.avreclipse.asm.option.include.paths"
                                    valueType="includePath"
                                >
                                        <ListOptionValue
                                            ...
                                            value="/usr/share/arduino/hardware/arduino/cores/arduino"
                                        />
                                        ...
                                </option>
                                ...
                            </tool>
			    ...
			</toolChain>
		    </folderInfo>
		    <sourceEntries>
                        <entry
                            excluding="libraries/RC_Channel/examples|libraries/Filter/examples|libraries/DataFlash/examples"
                            ...
                        />
                    </sourceEntries>
                </configuration>
            <storageMudule>
            ...
        </cconfiguration>
    </storageModule>
    ...
</cproject>
"""
