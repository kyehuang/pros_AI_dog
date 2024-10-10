#! /bin/bash

docker run -it --rm\
    -p 9090:9090\
    -v $(pwd)/AI_pkg:/workspaces/AI_pkg\
    ghcr.io/otischung/pros_ai_image\
    /bin/bash