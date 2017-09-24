num_ps = {
    # membrane names (labels)
    H = {env, linear_prev, linear, angular};

    structure = [env
            [linear_prev
                [linear ]linear
            ]linear_prev
            [angular ]angular
            #[height  ]height
    ]env;

    env = {
    };

    #env = {
        #var = {height_z, orientation_x, orientation_y, orientation_z, acc_x, acc_y, acc_z, rot_x, rot_y, rot_z}; # variables used in the production function
        #pr = { -> 1|lw};
    #};

    linear_prev = {
        var = {tf_pos_x_0};
        pr = { tf_pos_x_0 -> 1|tf_pos_x_prev };
    };

    linear = {
        var = {tf_pos_x_1, tf_pos_x_prev, target_linear_x, loop_freq_hz, kp_linear, linear_x};
        #pr = { linear_x + (kp_linear * (target_linear_x - ((abs(tf_pos_x_1) - abs(tf_pos_x_prev)) / 5)) * (linear_x < 1) ) -> 1|linear_x };
        #pr = { linear_x + (kp_linear * (target_linear_x - abs((abs(tf_pos_x_1) - abs(tf_pos_x_prev)) / 5)) *


        pr = { linear_x + (kp_linear * (target_linear_x - ((abs(tf_pos_x_1) - abs(tf_pos_x_prev)) * 10 ))) *
                    #(linear_x < 1) -> 1|linear_x };
        #pr = { linear_x + (kp_linear * (target_linear_x - ((abs(tf_pos_x_1) - abs(tf_pos_x_prev)) * 100 ))) *
                    (linear_x < 1) -> 1|linear_x };
        var0 = {0, 0, 0};
    };

    angular = {
        var = {gyro_z, angular_z, target_angular_z, kp_angular};
        pr = { kp_angular * (target_angular_z - gyro_z) -> 1|angular_z};
    };

    #height = {
        #var = {height_z};
        #pr = { 1.2 * atan2((trans_y_0) (trans_x_0)) + 0*angular_z-> 1|angular_z};
    #};
}
