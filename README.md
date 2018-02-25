# comp313p_stdr

Changed general_forward_search_algorithm.py and controller_base.py

controller_base.py checks if a series of waypoints is in a straight line and ignores the ones in the centre, speeds up performance drastically.
general_forward_search_algorithm.py now has a line of sight function from the theta* algorithm. Consolidates number of waypoints based on if they are obstructed or not, just like theta*.

