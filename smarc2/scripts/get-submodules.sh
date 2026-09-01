#!/bin/bash
# get the directories from the gitmodules file
# they come as "path = blah path = asdf"
dirs=$( cat .gitmodules | grep "path = $1")
# remove the 'path =' parts
dirs=${dirs//"path = "/}
# then we can feed this to git submodule~
echo "Updating submodules in: $dirs"
git submodule update --remote --init $dirs
echo "Done"