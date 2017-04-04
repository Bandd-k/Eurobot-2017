#!/usr/bin/env python

import sys
import time
import json
import math 

import robot_class
import states

import smach

# import roslib
# import smach_ros
# import rospy


def parse_strategy(strategy_text, config_dict):

    commands_nocomments = [[part.strip() for part in line.split(":")] # get rid of empty lines and comments
                    for line in strategy_text.split('\n') if len(line.strip()) != 0 and line.strip()[0] != '#']
    durs, actions, par_list = [], [] ,[]
    # ===============================
    def isfloat(val):
        try:
            float(val)
            return True
        except:
            return False
    ### convert str to float
    def str_to_float(par_vals):
        new_vals = []
        for val in par_vals:
            try:
                f = float(val)
                new_vals += [f]
            except:
                fl = 0
                check_strs = ['', 'pi', 'pi_12', 'pi_32', 'pi_34']
                check_vals = [None, math.pi, math.pi/2, 3*math.pi/2]
                for strs, vals in zip(check_str, check_vals):
                    if val == strs:
                        new_vals += [vals]; fl = 1; break
                if not fl:
                    new_vals += map(float, map_str_to_config(val))
        return new_vals
    ## revert according to color
    def map_str_to_config(string, config_dict=config_dict):
        ## TODO: special mapping if wrong string
        if string in config_dict["robot data"]:
            return config_dict["robot data"][string]
        elif string in config_dict:
            return config_dict[string]
        else:
            return None
    # ===============================
    for command in commands_nocomments:
        if isfloat(command[0]):
                command[0] = float(command[0])
        if len(command) == 2:
            durs += [command[0]]; actions += [command[1]]; par_list += [None]
            continue

        dur, action, pars = command
        pars = pars.split(',') # if it is a sequence of parameters in a command
        pars = map(lambda x: x.strip().split(' '), pars) # parsed any string not whitespace
        for i, par_vals in enumerate(pars):
                pars[i] = str_to_float(map(lambda x: x.strip(), par_vals))
        durs += [dur]; actions += [action]; par_list += [pars]
    
    return [durs, actions, par_list]

def create_fsm(robot, name, config_dict, strategy_names, parsed_strategies):

    game_fsm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                                  output_keys = [])

    ud = game_fsm.userdata
    ud.robot = robot
    ud.color = config_dict["color"]
    ud.data = config_dict[name]
    ud.start_time = None
    # TODO include beacons from npParticle or vice versa
    #ud.beacons =
    ud.pts_gain = 0
    elements_names = ["ro_ba", "ro_la", "mo_mo_ba", "mo_mo_mi", "mo_mo_fr",
            "mo_mu_ba", "mo_mu_fr", "mo_mu_mi", "cr_ba", "cr_fr", "cr_co"]
    ud.resources_field = dict(zip(elements_names, [4, 4] +  6*[1] + [5, 5, 20]))
    ud.resources_picked = dict(zip(["mo_mo", "mu_mo", "tit_or"], 3*[0]))
    ud.parsed_strategies = dict(zip(strategy_names, parsed_strategies))

    state_list = ["take rocket back", "take rocket lateral", #"take crater corner", 
            #"take crater front", "take crater back", "drop starting area 2",
            "drop slot front", "drop slot lateral", #"drop cargo bay", 
            "pass robotische"]

    with game_fsm:

        general_in_keys = ["robot", "color", "name", "data", "start_time", 
                    "pts_gain", "resources_field", "resources_picked", "parsed_strategies"]
        
        # Robot initializer with 
        input_keys = ["robot", "start_time", "data"]
        output_keys = input_keys + ["sterr"]
        outcomes = ["initialized"]
        transition_out = ["PLANNER"]

        smach.StateMachine.add('ROBOT INIT', states.RobotInit( 
                        outcomes=outcomes,
                        input_keys=input_keys,
                        output_keys=output_keys),
                        transitions=dict(zip(outcomes, transition_out)),
                        remapping=dict(zip(output_keys, output_keys)))

        # Main Planner (decision making tuned with prepared strategy)
        input_keys = general_in_keys[:]
        output_keys = input_keys + ["sterr"] + ["parameter"]
        outcomes = state_list + ["ended"]
        transition_out = map(lambda x: x.upper(), state_list) + ["succeeded"]

        smach.StateMachine.add('PLANNER', states.Planner( 
                        outcomes=outcomes,
                        input_keys=input_keys,
                        output_keys=output_keys),
                        transitions=dict(zip(outcomes, transition_out)),
                        remapping=dict(zip(output_keys, output_keys)))

        # Pass robotische at the start
        input_keys = general_in_keys[:] + ["points"]
        input_keys.remove("name")
        input_keys.remove("parsed_strategies")
        output_keys = ["sterr", "time", "robot"]
        outcomes = ["succeeded", "aborted"]
        transition_out = ["PLANNER", "PLANNER"]

        smach.StateMachine.add('PASS ROBOTISCHE', states.PassRobotische( 
                        outcomes=outcomes,
                        input_keys=input_keys,
                        output_keys=output_keys),
                        transitions=dict(zip(outcomes, transition_out)),
                        remapping=dict(zip(output_keys, output_keys)))

        # Take modules from rocket at back
        input_keys = general_in_keys[:] + ["n_modules", "points"]
        input_keys.remove("name")
        input_keys.remove("parsed_strategies")
        output_keys = ["sterr", "time", "robot"]
        outcomes = ["succeeded", "aborted"]
        transition_out = ["PLANNER", "PLANNER"]

        smach.StateMachine.add('TAKE ROCKET BACK', states.TakeRocketBack(
                        outcomes=outcomes,
                        input_keys=input_keys,
                        output_keys=output_keys),
                        transitions=dict(zip(outcomes, transition_out)),
                        remapping=dict(zip(output_keys, output_keys)))

        # Drop modules to front slots (diagonal)
        input_keys = general_in_keys[:] + ["n_modules", "points"]
        input_keys.remove("name")
        input_keys.remove("parsed_strategies")
        output_keys = ["sterr", "time", "robot"]
        outcomes = ["succeeded", "aborted"]
        transition_out = ["PLANNER", "PLANNER"]
 
        smach.StateMachine.add('DROP SLOT FRONT', states.DropSlotFront( 
                        outcomes=outcomes,
                        input_keys=input_keys,
                        output_keys=output_keys),
                        transitions=dict(zip(outcomes, transition_out)),
                        remapping=dict(zip(output_keys, output_keys)))

        # Take modules from lateral rocket
        input_keys = general_in_keys[:] + ["n_modules", "points"]
        input_keys.remove("name")
        input_keys.remove("parsed_strategies")
        output_keys = ["sterr", "time", "robot"]
        outcomes = ["succeeded", "aborted"]
        transition_out = ["PLANNER", "PLANNER"]

        smach.StateMachine.add('TAKE ROCKET LATERAL', states.TakeRocketLateral(
                        outcomes=outcomes,
                        input_keys=input_keys,
                        output_keys=output_keys),
                        transitions=dict(zip(outcomes, transition_out)),
                        remapping=dict(zip(output_keys, output_keys)))

        # Drop modules to lateral slots
        input_keys = general_in_keys[:] + ["n_modules", "points"]
        input_keys.remove("name")
        input_keys.remove("parsed_strategies")
        output_keys = ["sterr", "time", "robot"]
        outcomes = ["succeeded", "aborted"]
        transition_out = ["PLANNER", "PLANNER"]

        smach.StateMachine.add('DROP SLOT LATERAL', states.DropSlotLateral( 
                        outcomes=outcomes,
                        input_keys=input_keys,
                        output_keys=output_keys),
                        transitions=dict(zip(outcomes, transition_out)),
                        remapping=dict(zip(output_keys, output_keys)))
   
    #End of FSM description
    return game_fsm

if __name__ == "__main__":

    with open(sys.argv[1]) as config_f:
            config_dict = json.load(config_f)
    with open(sys.argv[2]) as strategy_f:
            parsed_strategy = parse_strategy(strategy_f.read(), config_dict)

    name = sys.argv[2].split(".")[0].split("_")[1]
    robot = robot_class.Robot()
    fsm = create_fsm(robot, name, config_dict, ["min"], [parsed_strategy])
    #rospy.init_node('FSM', anonymous=True)
    #sis = smach_ros.IntrospectionServer('server_name', fsm, '/SM_ROOT')
    #sis.start()
    ## Execute the state machine
    tmp  = time.time()
    fsm.execute()
    print 'FSM elapsed after: ', time.time() - tmp, ' sec'
    ## Wait for ctrl-c to stop the application
    #rospy.spin()
    #sis.stop()
