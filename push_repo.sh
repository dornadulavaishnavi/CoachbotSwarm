#!/bin/bash

LOCATION="/home/hanlin/Desktop/vaishnavi/Automation_testing/CoachbotSwarm"
GIT="/usr/bin/git"
BRANCH_NAME="main"

cd "$LOCATION"
"$GIT" add -A
"$GIT" commit -am "from script"
"$GIT" push
