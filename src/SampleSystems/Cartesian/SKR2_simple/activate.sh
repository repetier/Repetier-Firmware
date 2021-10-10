#!/bin/bash

# Simple Mac/Linux script to active the configuration for selected printer model

cp Configuration* ../../../Repetier/src 
sed -i "" 's/default_envs =.*/default_envs = SKR2/' ../../../Repetier/platformio.ini 
