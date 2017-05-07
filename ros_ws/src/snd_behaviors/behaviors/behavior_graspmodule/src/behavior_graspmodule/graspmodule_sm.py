#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_graspmodule')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from snd_flexbe_states.move_servo_state import MoveServoState
from snd_flexbe_states.pump_state import PumpState
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 02 2017
@author: Romain
'''
class GraspModuleSM(Behavior):
  '''
  Grasp a lunar module
  '''


  def __init__(self):
    super(GraspModuleSM, self).__init__()
    self.name = 'GraspModule'

    # parameters of this behavior

    # references to used behaviors

    # Additional initialization code can be added inside the following tags
    # [MANUAL_INIT]
        
        # [/MANUAL_INIT]

    # Behavior comments:



  def create(self):
    ARM_SERVO_LOW = 495
    ARM_SERVO_HIGH = 270
    GRASP_SERVO_CLOSE = 100
    GRASP_SERVO_OPEN = 600
    ARM_SERVO_TOPIC = '/arm_servo'
    GRASP_SERVO_TOPIC = '/grasp_servo'
    PUMP_TOPIC = '/pump'
    # x:412 y:44, x:8 y:248
    _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

    # Additional creation code can be added inside the following tags
    # [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


    with _state_machine:
      # x:80 y:46
      OperatableStateMachine.add('OpenGripper',
                    MoveServoState(servo_topic=GRASP_SERVO_TOPIC, servo_position=GRASP_SERVO_OPEN, wait_time=0.5),
                    transitions={'done': 'LowerArm'},
                    autonomy={'done': Autonomy.Off})

      # x:370 y:316
      OperatableStateMachine.add('CloseGripper',
                    MoveServoState(servo_topic=GRASP_SERVO_TOPIC, servo_position=GRASP_SERVO_CLOSE, wait_time=0.5),
                    transitions={'done': 'LiftArm'},
                    autonomy={'done': Autonomy.Off})

      # x:368 y:224
      OperatableStateMachine.add('LiftArm',
                    MoveServoState(servo_topic=ARM_SERVO_TOPIC, servo_position=ARM_SERVO_HIGH, wait_time=0.5),
                    transitions={'done': 'PumpOff'},
                    autonomy={'done': Autonomy.Off})

      # x:95 y:210
      OperatableStateMachine.add('PumpOn',
                    PumpState(topic=PUMP_TOPIC, state=True, wait_time=0.5),
                    transitions={'done': 'Wait200ms'},
                    autonomy={'done': Autonomy.Off})

      # x:385 y:119
      OperatableStateMachine.add('PumpOff',
                    PumpState(topic=PUMP_TOPIC, state=False, wait_time=0.5),
                    transitions={'done': 'finished'},
                    autonomy={'done': Autonomy.Off})

      # x:78 y:132
      OperatableStateMachine.add('LowerArm',
                    MoveServoState(servo_topic=ARM_SERVO_TOPIC, servo_position=ARM_SERVO_LOW, wait_time=0.5),
                    transitions={'done': 'PumpOn'},
                    autonomy={'done': Autonomy.Off})

      # x:99 y:318
      OperatableStateMachine.add('Wait200ms',
                    WaitState(wait_time=0.200),
                    transitions={'done': 'CloseGripper'},
                    autonomy={'done': Autonomy.Off})


    return _state_machine


  # Private functions can be added inside the following tags
  # [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
