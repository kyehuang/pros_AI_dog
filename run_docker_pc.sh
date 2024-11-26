#! /bin/bash

docker run -it --rm\
    --gpus all\
    -p 9090:9090\
    -v $(pwd)/AI_pkg:/workspaces/AI_pkg\
    ghcr.io/kyehuang/pros_ppo_env\
    /bin/bash