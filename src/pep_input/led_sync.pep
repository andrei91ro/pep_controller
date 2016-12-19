num_ps = {
    # membrane names (labels)
    H = {env, light};

    structure = [env
        [light ]light
    ]env;

    env = {
        var = {init, robotID_0};
        pr = {init * (robotID_0 == 0) -> 1|epuck2_signal};
        var0 = {1};
    };

    light = {
        var = {robotID_1, base_led, bcast1, epuck0_signal, epuck1_signal, epuck2_signal}; # variables used in the production function

        # bcast, base_led should be consumed so they do not accumulate over time
        pr = { 2 * (((robotID_1 == 0) * epuck2_signal) + ((robotID_1 == 1) * epuck0_signal) + ((robotID_1 == 2) * epuck1_signal)) + (base_led * 0) + (bcast1 * 0) -> 1|base_led + 1|bcast1};
        var0 = ();
    };

}

