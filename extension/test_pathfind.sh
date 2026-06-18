# ensure all environment variable definitions such as "VAR=VAL" are treated as
# "export VAR=VAL" so that environment variables are exported to child process
set -a

# set all the environment variables
source data/map2.config

# run the pathfind program
START='(6.3, 26.3)' DEST='(6.01, 22.2)' PATHOUT=data/path1.csv ./pathfind

START='(11.1, 5.6)' DEST='(38.6, 6.1)' PATHOUT=data/path2.csv ./pathfind

#no path
START='(23.2, 8.3)' DEST='(35.5, 22.7)' PATHOUT=data/path3.csv ./pathfind

START='(8.1, 17.5)' DEST='(2.8, 18.6)' PATHOUT=data/path4.csv ./pathfind

#no path
START='(6.7, 17.8)' DEST='(18.5, 14.1)' PATHOUT=data/path5.csv ./pathfind

START='(14.1, 9.6)' DEST='(36, 25)' PATHOUT=data/path6.csv ./pathfind

START='(35.9, 28.0)' DEST='(24.7, 3.6)' PATHOUT=data/path7.csv ./pathfind

# ensure "VAR=VAL" definitons will no longer be exported to child processes
set +a
