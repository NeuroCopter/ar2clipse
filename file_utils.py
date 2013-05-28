import os
import shutil

def get_confirmation(question, ask):
    """
    @param question: [yes/no] question to be asked
    @param ask: False => question is skipped and answer asumed to be "yes"
    @return: True <=> the question was answered with "yes" || getting a confirmation was disabled
    """
    return not ask or raw_input("%s [y/n] " %question) == "y"

def rm(path, ask = True):
    """
    like rm -r path
    @param path: directory tree / file / symlink to be removed
    @param ask: True => get confirmation
    @return: False <=> deleting was cancled
    """
    if os.path.lexists(path):
        if os.path.islink(path):
            if get_confirmation('delete symlink "%s"?' %path, ask):
                os.remove(path)
                return True
            else:
                return False
        elif os.path.isfile(path):
            if get_confirmation('delete file "%s"?' %path, ask):
                os.remove(path)
                return True
            else:
                return False
        else:
            if get_confirmation('delete directory "%s"?' %path, ask):
                shutil.rmtree(path)
                return True
            else:
                return False
    else:
        return True
            
def copytree(src_path, dst_path, ignore_ = None, ask = True):
    """
    copies a directory.
    @see: http://docs.python.org/2/library/shutil.html#shutil.copytree
    @param src_path: directory to be copied
    @param dst_path: destination
    @param ignore_: copy-blacklist callable
    @param ask: True => get confirmation
    @return: False <=> copying was cancled
    """
    if get_confirmation('copy directory "%s" to "%s"?' %(src_path, dst_path), ask):
        shutil.copytree(src_path, dst_path, ignore = ignore_)
        return True
    else:
        return False

def linktree(src_path, dst_path, ignore_ = None, ask = True):
    """
    "copy-links" a directory
    
    - creates directory dst_path
    - links every file / folder in src_path to dst_path (if not on blacklist)
    
    @param src_path: directory to be copy-linked
    @param dst_path: destination directory
    @param ignore_: blacklist.callable line in copytree
    @param ask: True => get confirmation
    @return: False <=> copy-linking was cancled
    """
    if get_confirmation('link files from "%s" into "%s"?' %(src_path, dst_path), ask):
        os.mkdir(dst_path)
        ignore = [] if ignore_ is None else ignore_(src_path, os.listdir(src_path))
        for f in os.listdir(src_path):
            if f not in ignore:
                os.symlink( os.path.join(src_path, f), os.path.join(dst_path, f) )
        return True
    else:
        return False

def copyfile(src_file, dst_path, ask = True):
    """
    @see: http://docs.python.org/2/library/shutil.html#shutil.copy
    @param src_file: file to be copied
    @param dst_path: distination
    @param ask: True => get confirmation
    @return: False <=> copying was cancled
    """
    if get_confirmation('copy file "%s" to "%s"?' %(src_file, dst_path), ask):
        shutil.copy(src_file, dst_path)
        return True
    else:
        return False

def linkfile(src, link_name, ask = True):
    """
    @see: http://docs.python.org/2/library/os.html#os.symlink
    @param ask: True => get confirmation
    @return: False <=> linking was cancled
    """
    if get_confirmation('create symlink "%s" <-- "%s"?' %(src, link_name), ask):
        os.symlink(src, link_name)
        return True
    else:
        return False

class copyonly:
    """
    inverts the behaviour of ignore pattern: only files that the pettern matches are copied
    @see: http://docs.python.org/2/library/shutil.html#shutil.ignore_patterns
    """
    def __init__(self, include_patterns):
        """
        @param includes: list 
        """
        self._include_patterns = include_patterns
        
    def __call__(self, dirpath, files):
        return set(files) - set( shutil.ignore_patterns(*self._include_patterns)(dirpath, files) )
    