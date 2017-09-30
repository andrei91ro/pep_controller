num_ps = {
    # membrane names (labels)
    H = {env, height_error, height, angular_error, angular};

    structure = [env
            [height_error
                [height ]height
            ]height_error
            [angular_error
                [angular ]angular
            ]angular_error
    ]env;

    env = {
    };

    #env = {
        #var = {height_z, orientation_x, orientation_y, orientation_z, acc_x, acc_y, acc_z, rot_x, rot_y, rot_z}; # variables used in the production function
        #pr = { -> 1|lw};
    #};

    height_error = {
        var = {target_height, tf_pos_z, error_integral_height_0};
        E = {e1, e2};
        pr = { target_height - tf_pos_z [e1 -> ] 1|error_height };
        pr = { 2 * (error_integral_height_0 + (target_height - tf_pos_z)) [e2 -> ] 1|error_integral_height_0 + 1|error_integral_height_1 };
        E0 = {9999, 9999}; #TODO define ALWAYS_ON value to avoid vars > enzymes at some point
    };

    height = {
        var = {kp_height, ki_height, error_height, error_integral_height_1, linear_z};

        pr = { (kp_height * error_height + ki_height * error_integral_height_1)/(0-15) -> 1|linear_z };
        #var0 = {0, 0, 0};
    };

    angular_error = {
        var = {gyro_z, target_angular_z, error_integral_angular_0, error_integral_angular_1};
        E = {e3, e4};
        pr = { target_angular_z - gyro_z [e3 -> ] 1|error_angular};
        pr = { 2 * (error_integral_angular_0 + (target_angular_z - gyro_z)) + error_integral_angular_1 * 0 [e4 -> ] 1|error_integral_angular_0 + 1|error_integral_angular_1};
        E0 = {9999, 9999}; #TODO define ALWAYS_ON value to avoid vars > enzymes at some point
    };

    angular = {
        var = {gyro_z, angular_z, target_angular_z, kp_angular, ki_angular, error_angular, error_integral_angular_1};
        pr = { kp_angular * error_angular + ki_angular * error_integral_angular_1 -> 1|angular_z};
    };
}
