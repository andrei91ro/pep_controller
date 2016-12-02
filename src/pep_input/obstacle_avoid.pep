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
    };

    left = {
        var = {prox0, prox1, prox2, cruiseSpeedLeft, wL0, wL1, wL2}; # variables used in the production function
        pr = {cruiseSpeedLeft + wL0*prox0 + wL1*prox1 + wL2*prox2 + lw*0 -> 1|lw};
        var0 = (0, 0, 0, 10, 0.2, 0.6, 1);
    };

    speed_left = {
        var = {lw}; # variables used in the production function
        #pr = {lw -> 1|lw};
        var0 = (0);
    };

    right = {
        var = {prox0_, prox1_, prox2_, cruiseSpeedRight, wR0, wR1, wR2}; # variables used in the production function
        pr = {cruiseSpeedRight + wR0*prox0_ + wR1*prox1_ + wR2*prox2_ + rw*0 -> 1|rw};
        var0 = (0, 0, 0, 10, 1, 0.6, 0.2);
    };

    speed_right = {
        var = {rw}; # variables used in the production function
        var0 = (0);
    };
}

