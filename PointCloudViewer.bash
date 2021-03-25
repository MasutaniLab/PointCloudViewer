#!/bin/bash
dir=`dirname $0`
name=`basename $0 .bash`
cd ${dir}
gnome-terminal --title=${name} -- build/src/${name}Comp &