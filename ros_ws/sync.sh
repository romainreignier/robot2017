#!/bin/bash

rsync -av --delete --exclude-from=exclude.txt  src/ rc@rc.local:robot_ws/src
