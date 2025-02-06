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

cargo run --release -p bicycle_corr_exp -- rrfc static $SAVE_DATA
cargo run --release -p bicycle_corr_exp -- rrfc dynamic $SAVE_DATA
cargo run --release -p quadcopter_corr_exp -- rrfc static $SAVE_DATA
cargo run --release -p quadcopter_corr_exp -- rrfc dynamic $SAVE_DATA

cargo run --release -p bicycle_nbd_exp -- rrfc astar static $SAVE_DATA
cargo run --release -p bicycle_nbd_exp -- rrfc astar dynamic $SAVE_DATA
cargo run --release -p quadcopter_nbd_exp -- rrfc astar static $SAVE_DATA
cargo run --release -p quadcopter_nbd_exp -- rrfc astar dynamic $SAVE_DATA