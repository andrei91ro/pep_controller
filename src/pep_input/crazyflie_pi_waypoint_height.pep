num_ps = {
    # membrane names (labels)
    H = {env, tf_save, height_error, height, height_error_prev};

    structure = [env
        [tf_save
            [height_error
                [height
                    [height_error_prev ]height_error_prev
                    [filter ]filter
                ]height
            ]height_error
        ]tf_save
    ]env;

    env = {};

    tf_save = {
        var = {tf_pos_x, tf_pos_save};
        #var0 = {3, 0};
        pr = { 2*(tf_pos_x * (tf_pos_save == 0) + tf_pos_save * (tf_pos_save != 0)) -> 1|tf_pos_save + 1|target_height };
    };

    height_error = {
        var = {target_height, x0};
        pr = { 2*(target_height - x0/1000)  -> 1|error_height_0 + 1|error_height_1 };
    };

    height = {
        var = {kp_height, ki_height, error_height_0, error_height_prev, height_cmd_prev,
            height_lowest, height_highest, height_raw, zDistance};
        var0 = {1, 0.05, 0, 0, 0};
        pr = { 2 * min( (max( (
            height_cmd_prev + (kp_height * (error_height_0 - error_height_prev)) + (ki_height * ((error_height_0 + error_height_prev)/2) )
            ) (height_lowest) )) (height_highest) ) + zDistance*0 -> 1|zDistance + 1|height_cmd_prev };
    };

    height_error_prev = {
        var = {error_height_1};
        pr = { error_height_1 -> 1|error_height_prev };
    };
}
