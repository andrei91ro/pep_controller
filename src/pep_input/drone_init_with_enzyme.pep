num_ps = {
    # membrane names (labels)
    H = {env, linear, angular};

    structure = [env
            [linear_prev
                [linear ]linear
            ]linear_prev
            [angular ]angular
            [height  ]height
    ]env;

    env = {
    };

    #env = {
        #var = {height_z, orientation_x, orientation_y, orientation_z, acc_x, acc_y, acc_z, rot_x, rot_y, rot_z}; # variables used in the production function
        #pr = { -> 1|lw};
    #};

    linear_prev = {
        var = {tf_pos_x_0, tf_pos_x_prev};
        pr = { 3*tf_pos_x_0 -> 1|tf_pos_x_prev + 2|e_prev };
    };

    linear = {
        var = {tf_pos_x_1, tf_pos_x_prev, linear_x};
        E = {e_prev};
        pr = { (0.2 - (tf_pos_x_1 - tf_pos_x_prev)) [e_prev ->] -> 1|linear_x};
        var0 = {0, 0, 0}
        E0 = {0}
    };

    angular = {
        var = {gyro_z, angular_z, target_angular_z};
        #pr = { 1.2 * atan2((trans_y_0) (trans_x_0)) + 0*angular_z-> 1|angular_z};
        pr = {(0.2 - gyro_z) -> 1|angular_z};
    };

    height = {
        var = {height_z};
        pr = { 1.2 * atan2((trans_y_0) (trans_x_0)) + 0*angular_z-> 1|angular_z};
    };
}
