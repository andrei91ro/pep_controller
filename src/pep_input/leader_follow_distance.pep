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
        var = {prox0_0, prox1_0, prox2_0, prox5_0, prox6_0, prox7_0, cruiseSpeedLeft, kL}; # variables used in the production function
        pr = {cruiseSpeedLeft + kL * (prox0_0 - prox7_0) + lw*0 -> 1|lw};
        #var0 = (0, 0, 0, 10, 0.2, 0.6, 1);
    };

    speed_left = {
        var = {lw}; # variables used in the production function
        #pr = {lw -> 1|lw};
        var0 = (0);
    };

    right = {
        var = {prox0_1, prox1_1, prox2_1, prox5_1, prox6_1, prox7_1, cruiseSpeedRight, kR}; # variables used in the production function
        pr = {cruiseSpeedRight + kR * (prox0_1 - prox7_1) + rw*0 -> 1|rw};
        #var0 = (0, 0, 0, 10, 1, 0.6, 0.2);
    };

    speed_right = {
        var = {rw}; # variables used in the production function
        var0 = (0);
    };
}

