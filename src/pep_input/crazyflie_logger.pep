num_ps = {
    # membrane names (labels)
    H = {env, height};

    structure = [env
        [height ]height
    ]env;

    env = {};

    height = {
        var = {x0, x1, x2, tf_pos_x, tf_pos_y, tf_pos_z, orientation_x, orientation_y, orientation_z, vx, vy, yawrate, zDistance};
        pr = { 1 -> 1|zDistance };
    };
}
