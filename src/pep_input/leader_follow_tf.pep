num_ps = {
    # membrane names (labels)
    #H = {env, left, speed_left, right, speed_right};
    H = {env, linear, angular};

    structure = [env
            [linear ]linear
            [angular ]angular
    ]env;

    env = {
    };

    angular = {
        var = {trans_x_0, trans_y_0, rot_z_0, angular_z};
        pr = { 1.2 * atan2((trans_y_0) (trans_x_0)) + 0*angular_z-> 1|angular_z};
    };

    linear = {
        var = {trans_x_1, trans_y_1, rot_z_1, linear_x};
        pr = { 20 * sqrt(trans_x_1^2 + trans_y_1^2) + 0*linear_x -> 1|linear_x};
    };
}

