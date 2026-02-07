#!/bin/bash

# Purpose:  this script supports calling `gcc` with -E option to create and show
# the early stage, pre-processor only output given a C source file to process.

# 2026-02-06 Note this script in KX132 driver work in a deep path with respect
# to the Zephyr RTOS source tree against which it is compiled.  The `-I` include
# paths defined near the end reflect how the script must be called in the same
# directory as the source file it is given as its first and only argument.

function set_include_paths()
{
LIBRARY_PATHS=-I/home/ted/projects/container-based/v2-driver/docker_cache/zephyr/include/zephyr \
-I/home/ted/projects/container-based/v2-driver/docker_cache/zephyr/include \
-I../../../../zephyr/include
}

# This script based on early not sufficient command:
# gcc -E $LIBRARY_PATHS ./kx132-1211.c

echo "starting,"
echo "Looking for a single argument as source file to process with 'gcc -E' . . ."
echo

# On localhost with a distinct Zephyr SDK installed . . .
# ZEPHYR_SDK=zephyr-sdk-0.16.5-1
# COMPILER_PATH=/opt/$ZEPHYR_SDK/arm-zephyr-eabi/bin
# COMPILER=arm-zephyr-eabi-gcc

# In container on host without Zephyr SDK . . .
ZEPHYR_SDK=zephyr-sdk-0.17.4
COMPILER_PATH=/opt/toolchains/$ZEPHYR_SDK/arm-zephyr-eabi/bin
COMPILER=arm-zephyr-eabi-gcc

GCC=$COMPILER_PATH/$COMPILER

$GCC -E \
-I/home/ted/projects/container-based/v2-driver/docker_cache/zephyr/include/zephyr \
-I/home/ted/projects/container-based/v2-driver/docker_cache/zephyr/include \
-I../../../../zephyr/include \
-I../../../../../build/zephyr/include/generated \
-I. \
-I../../../../v2-driver/build/zephyr/include/generated \
$1

echo "done."
echo

exit $?
