#!/usr/bin/env python

import os
import fnmatch

root_dir = "."
exts = ["c", "h", "cpp", "hpp", "py", "sh"]

def get_lines(filenames):
    """Gets the number of lines of code for a file extension or array of
    extensions"""

    total = 0
    for filename in filenames:
        for line in open(filename):
            total += 1
    return total

def get_files(directory, extension):
    """ Returns a list of files with the specified extension """

    files = []
    for root, dirnames, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, "*." + extension):
            files.append(os.path.join(root, filename))

    return files

print("Type\t\tLines\t\tNumFiles")
total_lines = 0
total_files = 0
for ext in exts:
    files = get_files(root_dir, ext)
    lines = get_lines(files)
    print(ext + "\t\t" + str(lines) + "\t\t" + str(len(files)))
    total_lines += lines
    total_files += len(files)

print("----------------------------------------------------")
print("total\t\t" + str(total_lines) + "\t\t" + str(total_files))
