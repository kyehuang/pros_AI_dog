#! /bin/bash

docker run -it --rm\
    --gpus all\
    -p 9090:9090\
    -v $(pwd)/AI_pkg:/workspaces/AI_pkg\
    registry.screamtrumpet.csie.ncku.edu.tw/kyehuang/pros_ppo_env:0.0.2\
    /bin/bash