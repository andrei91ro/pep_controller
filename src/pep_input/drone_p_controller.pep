num_ps = {
    # membrane names (labels)
    H = {env, linear, height_error, height, angular_error, angular};

    structure = [env
            [linear ]linear
            [height_error
                [height ]height
            ]height_error
            [angular_error
                [angular ]angular
            ]angular_error
    ]env;

    env = {};

    linear = {
        var = {target_linear_x, linear_x};
        pr = { target_linear_x + linear_x*0 -> 1|linear_x };
    };

    height_error = {
        var = {target_height, tf_pos_z};
        pr = { target_height - tf_pos_z  -> 1|error_height };
    };

    height = {
        var = {kp_height, error_height, height_0, linear_z};
        pr = { (kp_height * error_height) + height_0 + linear_z*0 -> 1|linear_z };
    };

    angular_error = {
        var = {gyro_z, target_angular_z};
        pr = { target_angular_z - gyro_z -> 1|error_angular };
    };

    angular = {
        var = {gyro_z, angular_z, kp_angular, angular_0, error_angular};
        pr = { kp_angular * error_angular + angular_0 + angular_z*0 -> 1|angular_z};
    };
}
