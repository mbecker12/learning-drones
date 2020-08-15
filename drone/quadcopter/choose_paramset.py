def get_paramset(paramset_num):
    # should choose and compare:
    # high, medium, low kp
    # high, medium, low kd
    # on stable param set, investigate what difference ki makes
    if paramset_num == "00":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 0.4
        ki_rol = 0.0
        kd_rol = 0.08
        kp_pit = 0.4
        ki_pit = 0.0
        kd_pit = 0.08
        kp_yaw = 0.1
        ki_yaw = 0.0
        kd_yaw = 0.1

    elif paramset_num == "01":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 0.4
        ki_rol = 0.0
        kd_rol = 0.12
        kp_pit = 0.4
        ki_pit = 0.0
        kd_pit = 0.12
        kp_yaw = 0.1
        ki_yaw = 0.0
        kd_yaw = 0.12

    elif paramset_num == "02":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 0.4
        ki_rol = 0.0
        kd_rol = 0.2
        kp_pit = 0.4
        ki_pit = 0.0
        kd_pit = 0.2
        kp_yaw = 0.1
        ki_yaw = 0.0
        kd_yaw = 0.15

    elif paramset_num == "10":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 0.9
        ki_rol = 0.05
        kd_rol = 0.08
        kp_pit = 0.9
        ki_pit = 0.05
        kd_pit = 0.08
        kp_yaw = 0.2
        ki_yaw = 0.05
        kd_yaw = 0.1

    elif paramset_num == "11":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 0.9
        ki_rol = 0.01
        kd_rol = 0.12
        kp_pit = 0.9
        ki_pit = 0.01
        kd_pit = 0.12
        kp_yaw = 0.2
        ki_yaw = 0.01
        kd_yaw = 0.12

    elif paramset_num == "12":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 0.9
        ki_rol = 0.1
        kd_rol = 0.2
        kp_pit = 0.9
        ki_pit = 0.1
        kd_pit = 0.2
        kp_yaw = 0.2
        ki_yaw = 0.1
        kd_yaw = 0.15

    elif paramset_num == "13":
        kp_x = 1.2
        ki_x = 0.2
        kd_x = 0.9
        kp_y = 1.2
        ki_y = 0.2
        kd_y = 0.9
        kp_z = 1.0
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 0.9
        ki_rol = 0.05
        kd_rol = 0.08
        kp_pit = 0.9
        ki_pit = 0.05
        kd_pit = 0.08
        kp_yaw = 0.3
        ki_yaw = 0.05
        kd_yaw = 0.12

    elif paramset_num == "20":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 1.5
        ki_rol = 0.01
        kd_rol = 0.08
        kp_pit = 1.5
        ki_pit = 0.01
        kd_pit = 0.08
        kp_yaw = 0.4
        ki_yaw = 0.01
        kd_yaw = 0.1

    elif paramset_num == "21":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 1.5
        ki_rol = 0.01
        kd_rol = 0.12
        kp_pit = 1.5
        ki_pit = 0.01
        kd_pit = 0.12
        kp_yaw = 0.4
        ki_yaw = 0.01
        kd_yaw = 0.12

    elif paramset_num == "22":
        kp_x = 0.9
        ki_x = 0.1
        kd_x = 0.8
        kp_y = 0.9
        ki_y = 0.1
        kd_y = 0.8
        kp_z = 0.8
        ki_z = 0.1
        kd_z = 0.6

        kp_rol = 1.5
        ki_rol = 0.0
        kd_rol = 0.2
        kp_pit = 1.5
        ki_pit = 0.0
        kd_pit = 0.2
        kp_yaw = 0.4
        ki_yaw = 0.01
        kd_yaw = 0.15

    # elif paramset_num == '1':
    #     kp_x = 0.9; ki_x = 0.1; kd_x = 0.8;
    #     kp_y = 0.9; ki_y = 0.1; kd_y = 0.8;
    #     kp_z = 0.8; ki_z = 0.1; kd_z = 0.6;

    #     kp_rol = 0.7; ki_rol = 0.1; kd_rol = 0.2;
    #     kp_pit = 0.7; ki_pit = 0.1; kd_pit = 0.2;
    #     kp_yaw = 0.2; ki_yaw = 0.01; kd_yaw = 0.2;
    # elif paramset_num == '2':
    #     kp_x = 0.9; ki_x = 0.1; kd_x = 0.8;
    #     kp_y = 0.9; ki_y = 0.1; kd_y = 0.8;
    #     kp_z = 0.8; ki_z = 0.1; kd_z = 0.6;

    #     kp_rol = 1.0; ki_rol = 0.1; kd_rol = 0.1;
    #     kp_pit = 1.0; ki_pit = 0.1; kd_pit = 0.1;
    #     kp_yaw = 0.2; ki_yaw = 0.01; kd_yaw = 0.1;
    # elif paramset_num == '3':
    #     kp_x = 0.9; ki_x = 0.1; kd_x = 0.8;
    #     kp_y = 0.9; ki_y = 0.1; kd_y = 0.8;
    #     kp_z = 0.8; ki_z = 0.1; kd_z = 0.6;

    #     kp_rol = 1.2; ki_rol = 0.1; kd_rol = 0.1;
    #     kp_pit = 1.2; ki_pit = 0.1; kd_pit = 0.1;
    #     kp_yaw = 0.2; ki_yaw = 0.01; kd_yaw = 0.1;

    # elif paramset_num == '4':
    #     kp_x = 0.9; ki_x = 0.1; kd_x = 0.8;
    #     kp_y = 0.9; ki_y = 0.1; kd_y = 0.8;
    #     kp_z = 0.8; ki_z = 0.1; kd_z = 0.6;

    #     kp_rol = 0.5; ki_rol = 0.1; kd_rol = 0.09;
    #     kp_pit = 0.5; ki_pit = 0.1; kd_pit = 0.09;
    #     kp_yaw = 0.2; ki_yaw = 0.01; kd_yaw = 0.1;
    else:
        #     kp_x = 0.9; ki_x = 0.1; kd_x = 0.8;
        #     kp_y = 0.9; ki_y = 0.1; kd_y = 0.8;
        #     kp_z = 0.8; ki_z = 0.1; kd_z = 0.6;

        #     kp_rol = 0.7; ki_rol = 0.1; kd_rol = 0.1;
        #     kp_pit = 0.7; ki_pit = 0.1; kd_pit = 0.1;
        #     kp_yaw = 0.2; ki_yaw = 0.01; kd_yaw = 0.1;

        # if paramset_num == 'default':
        raise Exception("Invalid Parameter Set")
    k_dict = {
        "kp_x": kp_x,
        "kp_y": kp_y,
        "kp_z": kp_z,
        "ki_x": ki_x,
        "ki_y": ki_y,
        "ki_z": ki_z,
        "kd_x": kd_x,
        "kd_y": kd_y,
        "kd_z": kd_z,
        "kp_rol": kp_rol,
        "kp_pit": kp_pit,
        "kp_yaw": kp_yaw,
        "ki_rol": ki_rol,
        "ki_pit": ki_pit,
        "ki_yaw": ki_yaw,
        "kd_rol": kd_rol,
        "kd_pit": kd_pit,
        "kd_yaw": kd_yaw,
    }

    return k_dict


def convert_paramset_2_float(d):
    """
    :param d: dictionary containing all 9 PID constants
    """

    kp_x = d["kp_x"]
    kp_y = d["kp_y"]
    kp_z = d["kp_z"]

    ki_x = d["ki_x"]
    ki_y = d["ki_y"]
    ki_z = d["ki_z"]

    kd_x = d["kd_x"]
    kd_y = d["kd_y"]
    kd_z = d["kd_z"]

    kp_rol = d["kp_rol"]
    kp_pit = d["kp_pit"]
    kp_yaw = d["kp_yaw"]

    ki_rol = d["ki_rol"]
    ki_pit = d["ki_pit"]
    ki_yaw = d["ki_yaw"]

    kd_rol = d["kd_rol"]
    kd_pit = d["kd_pit"]
    kd_yaw = d["kd_yaw"]

    return (
        kp_x,
        kp_y,
        kp_z,
        ki_x,
        ki_y,
        ki_z,
        kd_x,
        kd_y,
        kd_z,
        kp_rol,
        kp_pit,
        kp_yaw,
        ki_rol,
        ki_pit,
        ki_yaw,
        kd_rol,
        kd_pit,
        kd_yaw,
    )
