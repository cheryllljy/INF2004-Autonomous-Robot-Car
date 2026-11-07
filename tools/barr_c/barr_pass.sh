#!/bin/bash
# Apply the Barr-C 2018 mechanical pass to the files given (app_program only).
T=$(cd "$(dirname "$0")" && pwd)
for f in "$@"; do expand -t 8 "$f" > "$f.x" && cat "$f.x" > "$f" && rm "$f.x"; done
python3 $T/move_comments.py "$@"
python3 $T/barr_fix.py "$@" | awk '{s+=$2} END {print "barr_fix edits", s}'
clang-format --style=file:$T/../../app_program/.clang-format -i "$@"
