import rospy,sys
from utils.geometry import Vector2D
from utils.functions import *
from krssg_ssl_msgs.msg import point_2d
from krssg_ssl_msgs.msg import BeliefState
from krssg_ssl_msgs.msg import gr_Commands
from krssg_ssl_msgs.msg import gr_Robot_Command
from krssg_ssl_msgs.msg import BeliefState
from role import  GoToBall, GoToPoint, KickToPoint
from role import REFEREE
from multiprocessing import Process
from kubs import kubs
from krssg_ssl_msgs.srv import bsServer
from math import atan2,pi
from utils.functions import *
from tactics import Goalie
import multiprocessing
import threading
#pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)


import memcache
shared = memcache.Client(['127.0.0.1:11211'],debug=True)

def function(id_1,state,pub,cmd):
	kub1 = kubs.kubs(id_1,state,pub)
	print(id_1)
	kub1.update_state(state)
	g_fsm = REFEREE.ref(cmd)
	# g_fsm = GoToPoint.GoToPoint()
	g_fsm.add_kub(kub1)

	i=0
	while(i<6):
		if i== id_1:
			i= i+1
			continue
		kub = kubs.kubs(i,state,pub)
		kub.update_state(state)	
		g_fsm.add_kub(kub)
		i=i+1
	#print(kub1.kubs_id)

	# g_fsm.add_point(point=kub.state.ballPos,orient=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x-3000)))
	#g_fsm.add_theta(theta=normalize_angle(pi+atan2(state.ballPos.y,state.ballPos.x+3000)))
	# g_fsm.as_graphviz()
	# g_fsm.write_diagram_png()
	#print('something before spin')
	g_fsm.spin()

def main1(process_id,x):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            print("process 1")
	            function(1,state.stateB,pub,x)
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()	
def main2(process_id,x):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            print("process 2")
	            function(2,state.stateB,pub,x)
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()	
def main3(process_id,x):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            print("process 3")
	            function(3,state.stateB,pub,x)
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()
def main4(process_id,x):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            print("process 4")
	            function(4,state.stateB,pub,x)
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()
def main5(process_id,x):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            print("process 5")
	            function(5,state.stateB,pub,x)
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()
def main6(process_id,x):
	pub = rospy.Publisher('/grsim_data',gr_Commands,queue_size=1000)
	rospy.init_node('node' + str(process_id),anonymous=False)

	while True:
	    state = None
        # state=shared.get('state')
	    rospy.wait_for_service('bsServer',)
	    getState = rospy.ServiceProxy('bsServer',bsServer)
	    try:
	            state = getState(state)
	    except rospy.ServiceException, e:
	            print("chutiya")		
	    if state:
	            #print('lasknfcjscnajnstate',state.stateB.homePos)
	            #p2 = multiprocessing.Process(target=function2, args=(2,state.stateB, )) 
	            print("process 6")
	            function(0,state.stateB,pub,x)
	            # break
	            #p2.start()
	            #p1.join()
	            #p2.join()
	           # print('chal ja')
	            # break
	    #rospy.spin()
while True:
	x = None
	while not x:
		x = raw_input()

	p1 = multiprocessing.Process(target=main1, args=(1,x))
	p2 = multiprocessing.Process(target=main2, args=(2,x))
	p3 = multiprocessing.Process(target=main3, args=(3,x))
	p4 = multiprocessing.Process(target=main4, args=(4,x))
	p5 = multiprocessing.Process(target=main5, args=(5,x))
	p6 = multiprocessing.Process(target=main6, args=(6,x))		
	p1.start()
	p2.start()
	p3.start()
	p4.start()
	p5.start()
	p6.start()
	p1.join()
	p2.join()
	p3.join()
	p4.join()
	p5.join()
	p6.join()

#main1()

# rospy.Subscriber('/belief_state', BeliefState, BS_callback, queue_size=1000)