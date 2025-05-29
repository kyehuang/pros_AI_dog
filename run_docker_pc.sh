#! /bin/bash

docker run -it --rm\
    -p 9090:9090\
    -v $(pwd)/AI_pkg:/workspaces/AI_pkg\
    registry.screamtrumpet.csie.ncku.edu.tw/kyehuang/pros_ppo_env:0.0.5\
    /bin/bash