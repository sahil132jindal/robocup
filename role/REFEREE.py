import sys,rospy
import composite_behavior
import behavior
from utils.config import *
from utils.state_functions import *
import time
from enum import Enum
import logging
from role import pass_receive
from role import GoToBall, GoToPoint, KickToPoint, _GoToPoint_
from utils.geometry import *
from utils.functions import *
from krssg_ssl_msgs.srv import bsServer
import math
import time
rospy.wait_for_service('bsServer',)
getState = rospy.ServiceProxy('bsServer',bsServer)

class ref(behavior.Behavior):

	class State(Enum):

		halt = 1
		freekick_us = 2
		freekick_opp = 3
		penalty_us = 4
		penalty_opp = 5
		kickoff = 6

	def __init__(self,cmd):
		super(ref,self).__init__()

		self.kubs = []
		self.kicker = None
		self.receiver = None
		self.receiving_point = None
		self.pass_angle = None
		self.behavior_failed = False
		self.ball_kicked = False
		self.cmd = cmd 

		for state in ref.State:
			self.add_state(state,behavior.Behavior.State.running)

		self.add_transition(behavior.Behavior.State.start,ref.State.halt,
			lambda:self.halt(),"halt")
		self.add_transition(behavior.Behavior.State.start,ref.State.freekick_us,
			lambda:self.freekick_us(),"freekick_us")
		self.add_transition(behavior.Behavior.State.start,ref.State.freekick_opp,
			lambda:self.freekick_opp(),"freekick_opp")					
		self.add_transition(behavior.Behavior.State.start,ref.State.penalty_us,
			lambda:self.penalty_us(),"penalty_us")
		self.add_transition(behavior.Behavior.State.start,ref.State.penalty_opp,
			lambda:self.penalty_opp(),"penalty_opp")
		self.add_transition(behavior.Behavior.State.start,ref.State.kickoff,
			lambda:self.kickoff(),"kickoff")
		self.add_transition(ref.State.halt,behavior.Behavior.State.completed,
			lambda:True,"halt completed")
		self.add_transition(ref.State.freekick_us,behavior.Behavior.State.completed,
			lambda:True,"freekick_us completed")
		self.add_transition(ref.State.freekick_opp,behavior.Behavior.State.completed,
			lambda:True,"freekick_opp completed")
		self.add_transition(ref.State.penalty_us,behavior.Behavior.State.completed,
			lambda:True,"penalty_us completed")
		self.add_transition(ref.State.penalty_opp,behavior.Behavior.State.completed,
			lambda:True,"penalty_opp completed")
		self.add_transition(ref.State.kickoff,behavior.Behavior.State.completed,
			lambda:True,"kickoff completed")

		for state in ref.State:
			self.add_transition(state,behavior.Behavior.State.failed,
				lambda:self.behavior_failed,"failed")

	def ball_moving(self):
		if magnitute(self.receiver.state.ballVel) <= 50:
			return False
		return True

	def add_kub(self,kub):
		self.kubs += [kub]

	def halt(self):
		if self.cmd=='HALT':
			return 1
		else:
			return 0

	def freekick_us(self):
		if self.cmd=='FREEKICKUS':
			return 1
		else:
			return 0

	def freekick_opp(self):
		if self.cmd=='FREEKICKOPP':
			return 1
		else:
			return 0

	def penalty_us(self):
		if self.cmd=='PENALTYUS':
			return 1
		else:
			return 0
	
	def penalty_opp(self):
		if self.cmd=='PENALTYOPP':
			return 1
		else:
			return 0

	def kickoff(self		):
		if self.cmd=='KICKOFF':
			return 1
		else:
			return 0

	def on_enter_halt(self):
		print("Entering halt")
		pass

	def execute_halt(self):
		print("Executing halt")
# 		if len(self.kubs) != 2:
# 			self.behavior_failed = True
# 			return
# 		if dist(self.kubs[0].state.ballPos,self.kubs[0].get_pos())< dist(self.kubs[0].state.ballPos,self.kubs[1].get_pos()):
# 			self.passer = self.kubs[0]
# 			self.receiver = self.kubs[1]
# -			self.receiving_point = Vector2D(self.kubs[1].get_pos().x,self.kubs[1].get_pos().y)
# 		else:
# 			self.passer = self.kubs[1]
# 			self.receiver = self.kubs[0]
# 			self.receiving_point = Vector2D(self.kubs[0].get_pos().x,self.kubs[0].get_pos().y)

		# self.pass_angle = normalize_angle(self.pass_angle)
		for kub in self.kubs:
			kub.reset()
			kub.execute()
		# state = None
		# state = getState(state)
		print("halted")

	def on_exit_halt(self):
		pass

	def on_enter_freekick_us(self):
		print("Entering freekick_us")
		pass

	def execute_freekick_us(self):
		print("Executing freekick_us")
		for kub in self.kubs:
			kub.reset()
			kub.execute()
		g_fsm = GoToPoint.GoToPoint()
		state = None
		state = getState(state)
		state = state.stateB		
		for kub in self.kubs: 
			kub.update_state(state)
		closest = our_bot_closest_to_ball(state)
		print("CLOSEST BOT ID : ",closest)
		# time.sleep(200)
		index=0
		for kub in self.kubs:
			if kub.kubs_id==closest:
				break
			index= index+1	
		self.kick_angle = angle_diff(self.kubs[index].state.ballPos,Vector2D(6000,0))
		g_fsm.DISTANCE_THRESH = 2* BOT_BALL_THRESH
		target_point = getPointBehindTheBall(self.kubs[index].state.ballPos,normalize_angle(self.kick_angle+math.pi))
		g_fsm.add_kub(self.kubs[index])
		g_fsm.add_point(target_point,self.kick_angle)
		g_fsm.avoid_ball = True
		g_fsm.spin()
		self.behavior_failed = g_fsm.behavior_failed
		print("Positions taken. Now kicking ")
		g_fsm = KickToPoint.KickToPoint(Vector2D(4500,0))
		g_fsm.power = math.sqrt(dist(Vector2D(4500,0),self.kubs[index].state.ballPos)/6750.0)*5.0
		state = None
		state = getState(state)
		state = state.stateB
		g_fsm.add_kub(self.kubs[index])
		g_fsm.add_theta(self.kick_angle)
		g_fsm.spin()
		self.behavior_failed = g_fsm.behavior_failed
		if g_fsm.state == behavior.Behavior.State.completed:
			self.ball_kicked = True

	def on_exit_freekick_us(self):
		pass
	def on_enter_freekick_opp(self):
		print("Entering freekick_opp")
		pass

	def execute_freekick_opp(self):
		print("Executing freekick_opp")
		for kub in self.kubs:
			kub.reset()
			kub.execute()
		g_fsm = GoToPoint.GoToPoint()
		g_fsm.DISTANCE_THRESH  = 2*BOT_BALL_THRESH
		state = None
		state = getState(state)
		state = state.stateB		
		for kub in self.kubs: 
			kub.update_state(state)
		closest = our_bot_closest_to_ball(state)
		print("CLOSEST BOT ID : ",closest)
		# time.sleep(200)
		index=0
		for kub in self.kubs:
			if kub.kubs_id==closest:
				break
			index= index+1	
		oppkub = opp_bot_closest_to_ball(state)		
		self.kick_angle = angle_diff(self.kubs[index].state.ballPos,Vector2D(6000,0))
		target_point = getPointBehindTheBall(self.kubs[index].state.ballPos,normalize_angle(self.kick_angle+math.pi))
		g_fsm.add_kub(self.kubs[index])
		g_fsm.add_point(target_point,self.kick_angle)
		g_fsm.avoid_ball = True 
		print("kickangle ",self.kick_angle)
		print("normalised kickangle ",normalize_angle(self.kick_angle+math.pi))
		time.sleep(2)
		g_fsm.spin()
		self.behavior_failed = g_fsm.behavior_failed
		
	def on_exit_freekick_opp(self):
		pass

	def on_enter_waiting(self):
		print("Enter waiting")
		pass

	def execute_waiting(self):
		while True:
			state = None
			state = getState(state)
			state = state.stateB
			self.passer.update_state(state)
			self.receiver.update_state(state)
			if math.fabs(math.fabs(normalize_angle(self.receiver.get_theta()-self.pass_angle))-math.pi) < deg_2_radian(10.0):
				break

	def on_exit_waiting(self):
		pass

	def on_enter_kicking(self):
		print("Enter kicking")
		self.kick_power = self.kicking_power()
		pass

	def execute_kicking(self):
		g_fsm = KickToPoint.KickToPoint(self.receiving_point)
		g_fsm.power = self.kick_power
		state = None
		state = getState(state)
		state = state.stateB
		self.passer.update_state(state)
		self.receiver.update_state(state)
		g_fsm.add_kub(self.passer)
		g_fsm.add_theta(self.pass_angle)
		g_fsm.spin()
		self.behavior_failed = g_fsm.behavior_failed
		if g_fsm.state == behavior.Behavior.State.completed:
			self.ball_kicked = True

	def on_exit_kicking(self):
		print("exiting kicking")
		# # if not self.behavior_failed:
		# self.ball_kicked = True
		# # else:
		# # 	print("noooooooooooooooooooooooooooooooooooooooooooooooooooooo!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
		pass

	def on_enter_receiver(self):
		print("Entering receiver")
		pass

	def execute_receiver(self):
		print("Executing receiver")
		target_point = Vector2D(self.receiver.get_pos().x,self.receiver.get_pos().y)
		g_fsm = GoToPoint.GoToPoint()
		g_fsm.add_kub(self.receiver)
		g_fsm.add_point(target_point,normalize_angle(self.pass_angle+math.pi))
		g_fsm.spin()
		self.behavior_failed = g_fsm.behavior_failed

	def on_exit_receiver(self):
		print("exiting receiver")
		pass

	def on_enter_going_to_receiving_point(self):
		print("going to receiving point")
		self.receiving_point = self.calculate_receiving_point()
		# _GoToPoint_.init(self.receiver,self.receiving_point,self.receiver.get_theta())
		pass

	def execute_going_to_receiving_point(self):
		print("Executing receiving", self.passer.kubs_id, self.receiver.kubs_id)
		# start_time = rospy.Time.now()
		# start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)
		# generatingfunction = _GoToPoint_.execute(start_time,BOT_BALL_THRESH*2.0)
		# for gf in generatingfunction:
		# 	self.receiver,receiving_point = gf
		# 	if not vicinity_points(self.receiving_point,receiving_point,BOT_BALL_THRESH):
		# 		self.behavior_failed = True
		# 		break

		g_fsm = GoToPoint.GoToPoint()
		state = None
		state = getState(state)
		state = state.stateB
		self.passer.update_state(state)
		self.receiver.update_state(state)
		g_fsm.add_kub(self.receiver)
		g_fsm.add_point(self.receiving_point,self.receiver.get_theta())
		g_fsm.spin()
		self.behavior_failed = g_fsm.behavior_failed

		# while True:
		# 	state = None
		# 	state = getState(state)
		# 	state = state.stateB
		# 	self.passer.update_state(state)
		# 	self.receiver.update_state(state)
		# 	self.receiver.dribble = True
		# 	self.receiver.execute()
		# 	if self.receiver.has_ball():
		# 		break

	def on_exit_going_to_receiving_point(self):
		print("reached receiving point")
		pass

	def on_enter_receiving(self):
		print("entered receiving")
		pass

	def execute_receiving(self):
		while True:
			state = None
			state = getState(state).stateB
			self.receiver.update_state(state)
			self.receiver.dribble(True)
			self.receiver.execute()
			if self.ball_with_receiver() or not self.ball_moving():
				break

	def on_exit_receiving(self):
		print("ball received")
		pass

	def on_enter_received(self):
		print("Entered received")
		self.receiver.reset()
		pass

	def execute_received(self):
		self.receiver.execute()

	def on_exit_receiving(self):
		print("exiting received")
		pass







