string uuid                        # The uuid take from the tracker
string qtc_type                    # qtcbs|qtccs|qtcbcs|qtcbcs_arprobd
float64 quantisation_factor        # The quantisation factor for 0 states
float64 smoothing_rate             # Smoothed over X seconds
float64 distance_threshold         # Distance threshold for QTC_BC
string abstract_distance_threshold # Distance threshold for QTC_BC_ARGPROBD
bool validated                     # State chain has been validated
bool collapsed                     # Similar adjacent states have been collapsed
string qtc_robot_human             # QTC representation of type qtc_type, serialised using json
string prob_distance_robot_human   # arg_prob_distance relations, serialised using json
string qtc_goal_human              # QTC representation of type qtc_type, serialised using json
string prob_distance_goal_human    # arg_prob_distance relations, serialised using json
