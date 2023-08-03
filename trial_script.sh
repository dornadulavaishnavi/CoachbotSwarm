#!/bin/bash

LOCATION="/home/hanlin/Desktop/CoachbotSwarm"
GIT="/usr/bin/git"
BRANCH_NAME="main"

cd "$LOCATION"
"$GIT" add -A
"$GIT" commit -am "from script"
"$GIT" push
