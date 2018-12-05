__all__ = ['pyhackrf']

import os
import os.path as osp
import fnmatch

import logging
logging.basicConfig()
logger = logging.getLogger(osp.basename(__file__))
logger.setLevel(logging.WARNING)


# Directory retrieval utility.
def directoryUp(filename, levelsUp=1):
    """
    Get the path to the directory 'n' levels up.
    """
    filename = osp.dirname(osp.abspath(filename))
    if levelsUp <= 0:
        return osp.dirname(filename)


    levelStr = "/".join(["..",]*levelsUp)

    upDir = osp.abspath(osp.join(filename, levelStr))
    return upDir


# File search utility.
def findFile(pattern, levelsUp=2, directory=None):
    """
    Find a file with the corresponding name.

    Args:
        pattern (str):  Pattern of filename.
            Example: Can use wildcard to search for first filename that matches in directory search path.
        levelsUp (int): Number of directories 'up' from package directory.
        directory (str): Directory to start the search in (overrides 'levelsUp').

    Returns:
        First file that matches the pattern (str).
    """
    searchDir = directoryUp(__file__, levelsUp) if ((directory is None) or (levelsUp < 0)) else directory
    for root, dirs, files in os.walk(searchDir):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                return osp.join(root, name)

    return None


# Get path to 'libhackrf.so'
LIBHACKRF_PATH=None
if os.path.exists("/usr/lib/x86_64-linux-gnu/"):
    LIBHACKRF_PATH = findFile("libhackrf.so", directory="/usr/lib/x86_64-linux-gnu")
    if LIBHACKRF_PATH is None:
        logger.debug("\'libhackrf.so\' not in expected place...")
        logger.debug("    Searching \'/usr/lib\'...")
        LIBHACKRF_PATH = findFile("libhackrf.so", directory="/usr/lib")
        logger.debug("    Searching \'/home\'...")
        LIBHACKRF_PATH = findFile("libhackrf.so", directory="/home")


# Check if 'libhackrf.so' was found.
if LIBHACKRF_PATH is None:
    logger.error("Could not find \'libhackrf.so\', which is required to build the library.")
    raise Exception("Could not find \'libhackrf.so\', which is required to build the library.")
else:
    logger.debug("Found \'libhackrf.so\': \'%s\'." % LIBHACKRF_PATH)


# Open 'libhackrf.so' library so all modules can use the same instance.
import ctypes
LIBHACKRF_SO = ctypes.CDLL(LIBHACKRF_PATH)
