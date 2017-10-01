num_ps = {
    # membrane names (labels)
    H = {env, linear, height_error, height, height_error_prev, angular_error, angular_error_prev, angular};

    structure = [env
            [linear ]linear
            [height_error
                [height
                    [height_error_prev ]height_error_prev
                ]height
            ]height_error
            [angular_error
                [angular
                    [angular_error_prev ]angular_error_prev
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
        var = {kp_height, ki_height, error_height_0, linear_z, error_height_prev};
        pr = { (kp_height * error_height_0) + (ki_height * ((error_height_prev + error_height_0)/2) ) + linear_z*0 -> 1|linear_z };
    };

    height_error_prev = {
        var = {error_height_1};
        pr = { error_height_1 + 0*error_height_prev -> 1|error_height_prev };
    };

    angular_error = {
        var = {gyro_z, target_angular_z};
        pr = { 2*(target_angular_z - gyro_z) -> 1|error_angular_0 + 1|error_angular_1 };
    };

    angular = {
        var = {gyro_z, angular_z, kp_angular, ki_angular, error_angular_0, error_angular_prev};
        pr = { (kp_angular * error_angular_0) + (ki_angular * ((error_angular_prev + error_angular_0)/2) ) -> 1|angular_z};
    };

    angular_error_prev = {
        var = {error_angular_1};
        pr = { error_angular_1 + 0*error_angular_prev -> 1|error_angular_prev };
    };
}
