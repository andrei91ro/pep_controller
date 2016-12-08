num_ps = {
    # membrane names (labels)
    H = {env, left, speed_left, led_left, led_right, right, speed_right};

    structure = [env
        [left
            [speed_left ]speed_left
            [led_left ]led_left
        ]left

        [right
            [speed_right ]speed_right
            [led_right ]led_right
        ]right
    ]env;

    env = {
    };

    left = {
        var = {prox0, prox1, prox2, prox5, prox6, prox7, cruiseSpeedLeft, wL0, wL1, wL2, wL5, wL6, wL7}; # variables used in the production function
        pr = {cruiseSpeedLeft + wL0*prox0 + wL1*prox1 + wL2*prox2 + wL5*prox5 + wL6*prox6 + wL7*prox7 + lw*0 -> 1|lw};
        #var0 = (0, 0, 0, 10, 0.2, 0.6, 1);
    };

    speed_left = {
        var = {lw}; # variables used in the production function
        #pr = {lw -> 1|lw};
        var0 = (0);
    };

    led_left = {
        var = {led7}; # variables used in the production function
        pr = {0 - led7  -> 1|led7};
        var0 = (-1);
    };

    right = {
        var = {prox0_, prox1_, prox2_, prox5_, prox6_, prox7_, cruiseSpeedRight, wR0, wR1, wR2, wR5, wR6, wR7}; # variables used in the production function
        pr = {cruiseSpeedRight + wR0*prox0_ + wR1*prox1_ + wR2*prox2_ + wR5*prox5_ + wR6*prox6_ + wR7*prox7_ + rw*0 -> 1|rw};
        #var0 = (0, 0, 0, 10, 1, 0.6, 0.2);
    };

    speed_right = {
        var = {rw}; # variables used in the production function
        var0 = (0);
    };

    led_right = {
        #var = {led0}; # variables used in the production function
        #pr = {0 - led0  -> 1|led0};
        var = {led1}; # variables used in the production function
        pr = {0 - led1  -> 1|led1};
        var0 = (1);
    };
}

