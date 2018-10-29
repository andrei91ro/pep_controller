num_ps = {
    # membrane names (labels)
    H = {env, linear, height_error, height, height_error_integral, angular_error, angular_error_integral, angular};

    structure = [env
            [linear ]linear
            [height_error
                [height
                    [height_error_integral ]height_error_integral
                ]height
            ]height_error
            [angular_error
                [angular
                    [angular_error_integral ]angular_error_integral
                ]angular
            ]angular_error
    ]env;

    env = {};

    linear = {
        var = {target_linear_x, linear_x};
        pr = { target_linear_x + linear_x*0 -> 1|linear_x };
    };

    height_error = {
        var = {target_height, tf_pos_z};
        pr = { 2*(target_height - tf_pos_z)  -> 1|error_height_0 + 1|error_height_1 };
    };

    height = {
        var = {kp_height, ki_height, error_height_0, linear_z, error_height_integral};
        pr = { (kp_height * error_height_0) + (ki_height * error_height_integral) + linear_z*0 -> 1|linear_z };
    };

    height_error_integral = {
        var = {error_height_1, loop_rate_hz};
        pr = { error_height_1 * (1/loop_rate_hz) -> 1|error_height_integral };
    };

    angular_error = {
        var = {gyro_z, target_angular_z};
        pr = { 2*(target_angular_z - gyro_z) -> 1|error_angular_0 + 1|error_angular_1 };
    };

    angular = {
        var = {gyro_z, angular_z, kp_angular, ki_angular, error_angular_0, error_angular_integral};
        pr = { (kp_angular * error_angular_0) + (ki_angular * error_angular_integral ) -> 1|angular_z};
    };

    angular_error_integral = {
        var = {error_angular_1, loop_rate_hz};
        pr = { error_angular_1 * (1/loop_rate_hz) -> 1|error_angular_integral };
    };
}
