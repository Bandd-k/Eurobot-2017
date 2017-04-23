#!/usr/bin/env python

import sys
import time
import json
#from multiprocessing import Process, Queue, Value,Array

import robot_class
from hokuyolx import HokuyoLX
#import main_file

import smach

import roslib
import smach_ros
import rospy

# import logging
# logging.getLogger('rosout').setLevel(logging.CRITICAL)
# class Filter(logging.Filter):
#     def filter(sf, record):
#         return 'State machine transitioning' not in record.msg
#         return 'State machine transitioning' not in record.msg
# logging.getLogger('rosout').addFilter(Filter())


test_time = 0
duration = 2

def parse_strategy(strategy_text, config_dict):

    commands_nocomments = [[part.strip() for part in line.split(":")]
                for line in strategy_text.split('\n') if line[0] != '#']

    def parameter_parse(command):
        def isfloat(value): # check if value can be converted to float
            try:
                float(value)
                return True
            except:
                return False
        if len(command) == 1:
            return [command[0], None]
        pars = command[1].split(',') # if it is a sequence of parameters in a command
        pars = map(lambda x: x.strip().split(' '), pars) # parsed any string not whitespace

        for i, par_vals in enumerate(pars):
            if not isfloat(par_vals[0].strip()):# check if parameter was a string parameter
                pars[i] = config_dict[' '.join(par_vals).strip()] # use hardcoded value from game_conf json
            else:
                 pars[i] = [float(val) for val in par_vals]
        return [command[0], pars]

    return map(parameter_parse, commands_nocomments)

class RobotInit(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        #ud.robot = Robot(config_data=ud.robot_data)
        # checking for button in a loop...
        #time.sleep(1)

        ud.robot = robot_class.Robot(config_data=ud.robot_data)
        ud.start_time = time.time()
        #time.sleep(test_time)
        time.sleep(1)
        return 'robot initialized'


class Timer(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        #cnt = 10
        while 'start_time' not in ud:# and cnt:
            print('>>> TIMER Waiting for data...')
            #cnt -=1
            time.sleep(0.5)
        print('>>> TIMER GOT DATA! x = '+str(ud.start_time))
        time.sleep(ud.duration)
        return 'time elapsed'

class CommandHandler(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)
        sf.command_num = 0

    def execute(sf, ud):
        #time.sleep(test_time)
        #print 'Command handler userdata keys', ud.__getattr__()
        while 'robot' not in ud:
            print 'Command handler userdata keys', '\n'.join([str(key) + str(ud[key]) for key in ud.keys()])#__contains__()
            print('>>> COMMAND HANDLER Waiting for data...')
            time.sleep(0.5)
        if not sf.command_num:
            print('>>> COMMAND HANDLER GOT ROBOT!')
        if sf.command_num == len(ud.parsed_strategy):
            return 'strategy ended'
        if sf.preempt_requested():
            sf.service_preempt()
            return 'preempted'
        command = ud.parsed_strategy[sf.command_num]
        sf.command_num += 1
        #ud.action = command[0]
        ud.parameter = command[1]
        return command[0].lower()

    def request_preempt(sf):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(sf)
        #rospy.logwarn("Preempted!")

class Motion(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        time.sleep(test_time)
        return 'succeeded'

    def request_preempt(sf):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(sf)
        #rospy.logwarn("Preempted!")

class TakeCylinder(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        time.sleep(test_time)
        return 'succeeded'

class DropCylinder(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        time.sleep(test_time)
        return 'succeeded'

class CollisionHandler(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        time.sleep(test_time)
        ud.new_point = None
        return 'succeeded'
    def request_preempt(sf):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(sf)
        #rospy.logwarn("Preempted!")

class FunnyAction(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        time.sleep(test_time)
        return 'succeeded'

    def request_preempt(sf):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(sf)
        #rospy.logwarn("Preempted!")


# gets called when ANY child state terminates
def startup_child_term_cb(outcome_map):
    # terminate all running states if TIMER finished with outcome 'time elapsed'
    if outcome_map['TIMER'] == 'time elapsed':
        return True
    if outcome_map['ROBOT'] == 'aborted':
        return True
    # We need to wait for 90 seconds (when timer ends)
    # terminate all running states if ROBOT finished
    #if outcome_map['ROBOT']:
    #    return True
    # in all other case, just keep running, don't terminate anything
    return False


    # gets called when ALL child states are terminated
def startup_out_cb(outcome_map):
    if outcome_map['ROBOT'] == 'strategy ended':
        # print(outcome_map)
        return 'strategy succeeded'
    elif outcome_map['TIMER'] == 'time elapsed':
        return 'timer ended'
    elif outcome_map['ROBOT'] == 'aborted':
        return 'robot aborted'
    else:
        return 'strategy succeeded'


def create_fsm():

    game_fsm = smach.StateMachine(outcomes=['succeeded', 'aborted'],
                                  output_keys = [])
    with game_fsm:

        robot_strategy_fsm = smach.Concurrence(
            outcomes=['timer ended', 'strategy succeeded', 'robot aborted'],
            default_outcome='timer ended',
            input_keys=[],
            output_keys=[],
            child_termination_cb = startup_child_term_cb, # use instead of outcome_map
            outcome_cb = startup_out_cb
        )

        with open(sys.argv[1]) as config_f:
            config_dict = json.load(config_f)

        robot_strategy_ud = robot_strategy_fsm.userdata
        robot_strategy_ud.robot_data = config_dict['robot data']
        actions =[action_str.encode('ascii','ignore') for action_str in config_dict['strategy actions']]
        robot_strategy_ud.game_field = config_dict['game field']

        with open(sys.argv[2]) as f_rob_strategy:
            robot_strategy_ud.parsed_strategy = parse_strategy(f_rob_strategy.read(),robot_strategy_ud.robot_data)

        robot_strategy_ud.duration = sys.argv[3] if len(sys.argv) > 3 else duration # 90 default seconds duration of the game timer
        #startup_ud.start_time = None

        smach.StateMachine.add('ROBOT STRATEGY', robot_strategy_fsm,
                               transitions={'timer ended': 'FUNNY ACTION',
                                            'strategy succeeded': 'FUNNY ACTION',
                                            'robot aborted': 'aborted'})

        smach.StateMachine.add('FUNNY ACTION', FunnyAction(outcomes=['succeeded', 'aborted'],
                                                           input_keys=['robot', 'parameter', 'state'],
                                                           output_keys=[]),
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted'})


        with robot_strategy_fsm:

            # Here we initialize and wait the end of it
            smach.Concurrence.add('ROBOT INIT', RobotInit(outcomes=['robot initialized', 'initialization failed'],
                                                               input_keys=['robot_data', 'parsed_strategy'],#['robot_data']
                                                               output_keys=['start_time', 'robot', 'parsed_strategy']))
                                       # transitions={'robot initialized': 'ROBOT STRATEGY',
                                       #              'initialization failed': 'aborted'})

            smach.Concurrence.add('TIMER', Timer(outcomes=['time elapsed'],
                                                 input_keys=['start_time', 'duration'],
                                                 output_keys=['deadline1']))


            sm_robot = smach.StateMachine(outcomes=['aborted', 'strategy ended', 'preempted'],
                                          input_keys = ['game_field', 'parsed_strategy', 'robot'],
                                          output_keys=[])

            smach.Concurrence.add('ROBOT', sm_robot)




            with sm_robot:
                # Command Hadler

                # Actions as states
                smach.StateMachine.add('TAKE CYLINDER', TakeCylinder(outcomes=['succeeded', 'aborted'],
                                                                 input_keys=['parameter', 'robot'],
                                                                 output_keys=[]),
                                       #state = 0, 1, 2  0 - succeded, 1 - aborted
                                       transitions={'succeeded': 'COMMAND HANDLER',
                                                    'aborted': 'COMMAND HANDLER'})

                smach.StateMachine.add('DROP CYLINDER', DropCylinder(outcomes=['succeeded', 'aborted'],
                                                                 input_keys=['parameter', 'robot'],
                                                                 output_keys=[]),
                                       #state = 0, 1, 2  0 - succeded, 1 - aborted
                                       transitions={'succeeded': 'COMMAND HANDLER',
                                                    'aborted': 'COMMAND HANDLER'})

                motion_fsm = smach.Concurrence(
                    outcomes=['succeeded', 'aborted'],
                    default_outcome='succeeded',
                    input_keys=['point', 'robot'], # parameter remapping
                    output_keys=['new_point', 'robot'],
                    #TODO#        child_termination_cb = child_term_cb, // use instead of outcome_map
                    #TODO#        outcome_cb = out_cb,
                    outcome_map = {
                        'succeeded': {'COLLISION HANDLER': 'succeeded', 'MOTION': 'succeeded'},
                        'aborted': {'COLLISION HANDLER': 'aborted'},
                        #'aborted': {'ROBOT': 'aborted'}
                    }
                )

                smach.StateMachine.add('MOVE', motion_fsm,
                                            transitions={'succeeded': 'COMMAND HANDLER',
                                                    'aborted': 'COMMAND HANDLER'},
                                            remapping = {'point':'parameter'})

                with motion_fsm:
                    smach.Concurrence.add('MOTION', Motion(outcomes=['succeeded', 'aborted'],
                                                            input_keys=['point', 'robot'],
                                                            output_keys=[]))
                    smach.Concurrence.add('COLLISION HANDLER', CollisionHandler(outcomes=['succeeded', 'aborted'],
                                                                               input_keys=['point', 'robot'],
                                                                               output_keys=['new_point']))

                command_handler_trans = {action:action.upper() for action in actions} # commands parsing for Command handler
                command_handler_trans.update({'strategy ended':'strategy ended', 'preempted':'preempted'})

                smach.StateMachine.add('COMMAND HANDLER', CommandHandler(outcomes=actions + ['strategy ended'] + ['preempted'],
                                                                         input_keys=['parsed_strategy', 'robot'],
                                                                         output_keys=['parameter', 'robot']),
                                       transitions=command_handler_trans)

            # End Robot states initialisation, except Funny action
    #End of FSM description
    return game_fsm

if __name__ == "__main__":
    tmp  = time.time()
    fsm = create_fsm()
    rospy.init_node('FSM', anonymous=True)
    sis = smach_ros.IntrospectionServer('server_name', fsm, '/SM_ROOT')
    sis.start()
    # Execute the state machine
    fsm.execute()
    # Wait for ctrl-c to stop the application
    #rospy.spin()
    sis.stop()
    print 'FSM elapsed after: ', time.time() - tmp, ' sec'
