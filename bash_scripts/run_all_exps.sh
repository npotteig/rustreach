#/bin/bash

# Check if an argument is provided
if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <0 or 1>"
    return
fi

# Validate the argument (must be 0 or 1)
if [[ "$1" != "0" && "$1" != "1" ]]; then
    echo "Error: Argument must be 0 or 1."
    return
fi

# Save the input to a variable
SAVE_DATA=$1

source bash_scripts/run_wo_exps.sh $SAVE_DATA
source bash_scripts/run_rrfc_exps.sh $SAVE_DATA
source bash_scripts/run_rrrlc_exps.sh $SAVE_DATA