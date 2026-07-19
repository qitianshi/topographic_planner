if [[ $# -ne 1 ]]; then
    echo "Usage: ./animate.sh <config-file>" >&2
    exit 2
fi

# ensure all environment variable definitions such as "VAR=VAL" are treated as
# "export VAR=VAL" so that environment variables are exported to child process
set -a

# set all the environment variables
source $1

# ensure "VAR=VAL" definitons will no longer be exported to child processes
set +a

# run the animate program
./animate
