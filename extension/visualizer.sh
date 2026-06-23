# ensure all environment variable definitions such as "VAR=VAL" are treated as
# "export VAR=VAL" so that environment variables are exported to child process
set -a

# set all the environment variables
source data/pathfinder.config

# ensure "VAR=VAL" definitons will no longer be exported to child processes
set +a

# run the pathfind program
./visualizer 
