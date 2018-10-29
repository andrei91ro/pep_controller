num_ps = {
    # membrane names (labels)
    H = {env, led_left, led_right};

    structure = [env
        [led_left ]led_left
        [led_right ]led_right
    ]env;

    env = {
    };


    led_left = {
        var = {led7}; # variables used in the production function
        pr = {0 - led7  -> 1|led7};
        var0 = (-1);
    };


    led_right = {
        var = {led1}; # variables used in the production function
        pr = {0 - led1  -> 1|led1};
        var0 = (1);
    };
}

