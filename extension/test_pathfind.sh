# Script for testing multiple points simultaneously.

# enable echoing of commands
set -x

# ensure all environment variable definitions such as "VAR=VAL" are treated as
# "export VAR=VAL" so that environment variables are exported to child process
set -a

# set all the environment variables
source data/pathfinder.config

# ensure "VAR=VAL" definitons will no longer be exported to child processes
set +a

# run the pathfind program
START='(6.3, 26.3)' DEST='(6.01, 22.2)' PATHOUT=data/path1.csv ./pathfind

START='(11.1, 5.6)' DEST='(38.6, 6.1)' PATHOUT=data/path2.csv ./pathfind

# Expect no path
START='(23.2, 8.3)' DEST='(35.5, 22.7)' PATHOUT=data/path3.csv ./pathfind

START='(8.1, 17.5)' DEST='(2.8, 18.6)' PATHOUT=data/path4.csv ./pathfind

# Expect no path
START='(6.7, 17.8)' DEST='(18.5, 14.1)' PATHOUT=data/path5.csv ./pathfind

START='(14.1, 9.6)' DEST='(36, 25)' PATHOUT=data/path6.csv ./pathfind

START='(35.9, 28.0)' DEST='(24.7, 3.6)' PATHOUT=data/path7.csv ./pathfind

# Expect no path
START='(5.2, 14.3)' DEST='(29.1, 22.5)' PATHOUT=data/path8.csv ./pathfind

START='(34.5, 18.1)' DEST='(12.2, 5.5)' PATHOUT=data/path9.csv ./pathfind

# Expect no path
START='(7.8, 3.5)' DEST='(36.2, 24.8)' PATHOUT=data/path10.csv ./pathfind

START='(21.4, 26.2)' DEST='(15.7, 9.1)' PATHOUT=data/path11.csv ./pathfind

START='(11.9, 8.7)' DEST='(33.4, 13.6)' PATHOUT=data/path12.csv ./pathfind


# disable echoing of commands
set +x
