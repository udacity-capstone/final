
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
    	# TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
        	return 0.,0.,0.
		else:
			return 1., 0., 0.
