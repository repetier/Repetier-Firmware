#!/bin/bash

# Simple Mac/Linux script to active the configuration for selected printer model

cp Configuration* ../../../../Repetier/src 
PS3='Select processor variant: '
options=("Octopus 1.1 446" "Octopus 1.1 407")
select opt in "${options[@]}"
do
    echo "Selected: $opt"
    case $opt in
        "Octopus 1.1 446")
            sed -i "" 's/default_envs =.*/default_envs = BTT_OCTOPUS_446/' ../../../../Repetier/platformio.ini 
            break 
            ;;
        "Octopus 1.1 407")
            sed -i "" 's/default_envs =.*/default_envs = BTT_OCTOPUS_407/' ../../../../Repetier/platformio.ini 
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done


PS3='Select printer: '
options=("STACKER_F_SERIES_MODEL_1" "STACKER_F_SERIES_MODEL_2" "STACKER_F_SERIES_MODEL_2_IDEX" "STACKER_G2")
select opt in "${options[@]}"
do
    case $opt in
        "STACKER_F_SERIES_MODEL_1")
            sed -i "" 's/\/\/ #define STACKER_F_SERIES_MODEL_1/#define STACKER_F_SERIES_MODEL_1/' ../../../../Repetier/src/Configuration.h 
            break 
            ;;
        "STACKER_F_SERIES_MODEL_2")
            sed -i "" 's/\/\/ #define STACKER_F_SERIES_MODEL_2/#define STACKER_F_SERIES_MODEL_2/' ../../../../Repetier/src/Configuration.h 
            break 
            ;;
        "STACKER_F_SERIES_MODEL_2_IDEX")
            sed -i "" 's/\/\/ #define STACKER_F_SERIES_MODEL_2_IDEX/#define STACKER_F_SERIES_MODEL_2_IDEX/' ../../../../Repetier/src/Configuration.h
            break 
            ;;
        "STACKER_G2")
            sed -i "" 's/\/\/ #define STACKER_G2/#define STACKER_G2/' ../../../../Repetier/src/Configuration.h 
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done
