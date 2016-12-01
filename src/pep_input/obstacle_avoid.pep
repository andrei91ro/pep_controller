num_ps = {
    # membrane names (labels)
    H = {env, left, speed_left, right, speed_right};

    structure = [env
        [left
            [speed_left ]speed_left
        ]left

        [right
            [speed_right ]speed_right
        ]right
    ]env;

    env = {
        var = {prox0, prox1, prox2}; # variables used in the production function
        var0 = (0, 0, 0);
    };

    left = {
        var = {cruiseSpeedLeft, wL0, wL1, wL2}; # variables used in the production function
        pr = {cruiseSpeedLeft + wL0*prox0 + wL1*prox1 + wL2*prox2 + lw*0 -> 1|lw};
        var0 = (10, 0.2, 0.6, 1);
    };

    speed_left = {
        var = {lw}; # variables used in the production function
        #pr = {lw -> 1|lw};
        var0 = (0);
    };

    right = {
        var = {cruiseSpeedRight, wR0, wR1, wR2}; # variables used in the production function
        pr = {cruiseSpeedRight + wR0*prox0 + wR1*prox1 + wR2*prox2 -> 1|rw};
        var0 = (10, 0.2, 0.6, 1);
    };

    speed_right = {
        var = {rw}; # variables used in the production function
        pr = {rw -> 1|rw};
        var0 = (0);
    };
}

