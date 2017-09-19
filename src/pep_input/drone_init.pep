num_ps = {
    # membrane names (labels)
    H = {env};

    structure = [env
    ]env;

    env = {
        var = {height_z, orientation_x, orientation_y, orientation_z, acc_x, acc_y, acc_z, rot_x, rot_y, rot_z}; # variables used in the production function
        #pr = {cruiseSpeedLeft + wL0*prox0 + wL1*prox1 + wL2*prox2 + wL5*prox5 + wL6*prox6 + wL7*prox7 + lw*0 -> 1|lw};
    };
}
