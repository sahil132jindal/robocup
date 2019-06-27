from enum import Enum
import behavior
import _GoToPoint_
import rospy
from utils.functions import *
from utils.geometry import *
from utils.config import *
from math import *
import time

first = 0

startpoint = Vector2D(-4000,-1000)

side = 3000

class Square(behavior.Behavior):
    """docstring for Square"""
    class State(Enum):
        setup = 1 
        up = 2
        right = 3
        down = 4
        left = 5

    def __init__(self,course_approch_thresh =  DISTANCE_THRESH/3,continuous=False):

        super(Square,self).__init__()

        self.name = "Square"

        self.power = 7.0

        self.target_point = Vector2D()
        
        self.start_point = startpoint

        self.course_approch_thresh = course_approch_thresh

        self.ball_dist_thresh = 2*BOT_BALL_THRESH

        self.behavior_failed = False


        self.add_state(Square.State.setup,
            behavior.Behavior.State.running)

        self.add_state(Square.State.up,
            behavior.Behavior.State.running)
        
        self.add_state(Square.State.left,
            behavior.Behavior.State.running)

        self.add_state(Square.State.down,
            behavior.Behavior.State.running)

        self.add_state(Square.State.right,
            behavior.Behavior.State.running)
        
        self.add_transition(behavior.Behavior.State.failed,
            Square.State.setup,lambda: True,'immediately')

        self.add_transition(behavior.Behavior.State.start,
            Square.State.setup,lambda: True,'immediately')

        self.add_transition(Square.State.setup,
            Square.State.up,lambda: self.up(),'up')

        self.add_transition(Square.State.setup,
            Square.State.left,lambda:self.left(),'left')

        self.add_transition(Square.State.setup,
            Square.State.down,lambda: self.down(),'down')

        self.add_transition(Square.State.setup,
            Square.State.right,lambda:self.right(),'right')        

        self.add_transition(Square.State.up,
            Square.State.right,lambda:self.at_up_point() ,'right')

        self.add_transition(Square.State.left,
            Square.State.up,lambda:self.at_left_point() ,'up')

        self.add_transition(Square.State.down,
            Square.State.left,lambda:self.at_down_point(),'left')

        self.add_transition(Square.State.right,
            Square.State.down,lambda:self.at_right_point(),'down')

        #These three conditions for fail might cause a problem in dynamic gameplay as we are sending it back to setup and going to new point.
        self.add_transition(Square.State.up,
            Square.State.setup,lambda: self.behavior_failed,'failed')

        self.add_transition(Square.State.down,
            Square.State.setup,lambda: self.behavior_failed,'failed')

        self.add_transition(Square.State.right,
            Square.State.setup,lambda: self.behavior_failed,'failed')

        self.add_transition(Square.State.left,
            Square.State.setup,lambda: self.behavior_failed,'failed')

    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta

    def up(self):
        point=Vector2D(-4000,-1000)
        d4=dist(point,self.kub.get_pos())
        point=Vector2D(-4000,-1000+side)
        d1=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000)
        d2=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000+side)
        d3=dist(point,self.kub.get_pos())
        if d1<=d2 and d1<=d3 and d1<=d4:
            return True
        return False
    def right(self):
        point=Vector2D(-4000,-1000)
        d4=dist(point,self.kub.get_pos())
        point=Vector2D(-4000,-1000+side)
        d1=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000)
        d2=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000+side)
        d3=dist(point,self.kub.get_pos())
        if d3<=d1 and d3<=d2 and d3<=d4:
            return True
        return False
    def down(self):
        point=Vector2D(-4000,-1000)
        d4=dist(point,self.kub.get_pos())
        point=Vector2D(-4000,-1000+side)
        d1=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000)
        d2=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000+side)
        d3=dist(point,self.kub.get_pos())
        if d2<d3 and d2<d1 and d2<d4:
            return True
        return False
    def left(self):
        point=Vector2D(-4000,-1000)
        d4=dist(point,self.kub.get_pos())
        point=Vector2D(-4000,-1000+side)
        d1=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000)
        d2=dist(point,self.kub.get_pos())
        point=Vector2D(-4000+side,-1000+side)
        d3=dist(point,self.kub.get_pos())
        if d4<=d2 and d4<=d3 and d4<d1:
            return True
        return False
    # def target_present(self):
    #     return not ball_in_front_of_bot(self.kub) and self.target_point is not None 

    def at_up_point(self):
        p=Vector2D(self.start_point.x,self.start_point.y+side)
        return vicinity_points(p,self.kub.get_pos(),thresh= self.course_approch_thresh*2)
    def at_right_point(self):
        p=Vector2D(self.start_point.x+side,self.start_point.y+side)
        return vicinity_points(p,self.kub.get_pos(),thresh= self.course_approch_thresh*2)
    def at_down_point(self):
        p=Vector2D(self.start_point.x+side,self.start_point.y)
        return vicinity_points(p,self.kub.get_pos(),thresh= self.course_approch_thresh*2)
    def at_left_point(self):
        p=Vector2D(self.start_point.x,self.start_point.y)
        return vicinity_points(p,self.kub.get_pos(),thresh= self.course_approch_thresh*2)

    def at_target_point(self):
        return vicinity_points(self.target_point,self.kub.get_pos(),thresh= self.course_approch_thresh)

    def terminate(self):
        super().terminate()
        
    def on_enter_setup(self):
        self.behavior_failed = False
        pass
    def execute_setup(self):
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_up(self):
        theta = self.kub.get_pos().theta
        self.target_point.x=self.start_point.x
        self.target_point.y=self.start_point.y+side
        _GoToPoint_.init(self.kub, self.target_point, self.theta)
        #time.sleep(5)
        pass
        
    def execute_up(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
        #self.behavior_failed = False
        time.sleep(5)

        prevpoint=Vector2D(self.kub.get_pos())
        nextpoint=Vector2D(self.kub.get_pos())
        for gf in generatingfunction:
            prevpoint=nextpoint
            self.kub,target_point = gf
            nextpoint=self.kub.get_pos()
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            if not vicinity_points(prevpoint,nextpoint,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                self.kub.move(0,0)
                self.kub.execute()
                _GoToPoint_.init(self.kub, self.kub.get_pos(), self.theta)
                print 'BREAKINGGGG'
                break

        
    def on_exit_up(self):
        self.kub.move(0,0)
        self.kub.execute()
        pass

    def on_enter_right(self):
        theta = self.kub.get_pos().theta
        self.target_point.x=self.start_point.x+side
        self.target_point.y=self.start_point.y+side
        _GoToPoint_.init(self.kub, self.target_point, theta)
        #time.sleep(5)
        pass
        
    def execute_right(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
        #self.behavior_failed = False
        time.sleep(5)
        prevpoint=Vector2D(self.kub.get_pos())
        nextpoint=Vector2D(self.kub.get_pos())
        for gf in generatingfunction:
            prevpoint=nextpoint
            self.kub,target_point = gf
            nextpoint=self.kub.get_pos()
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            if not vicinity_points(prevpoint,nextpoint,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                self.kub.move(0,0)
                self.kub.execute()
                _GoToPoint_.init(self.kub, self.kub.get_pos(), self.theta)
                print 'BREAKINGGGG'
                break
        
    def on_exit_right(self):
        self.kub.move(0,0)
        self.kub.execute()
        pass

    def on_enter_down(self):
        theta = self.kub.get_pos().theta
        self.target_point.x=self.start_point.x+side
        self.target_point.y=self.start_point.y
        _GoToPoint_.init(self.kub, self.target_point, theta)
        #time.sleep(5)
        
        pass
        
    def execute_down(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
        #self.behavior_failed = False
        time.sleep(5)
        prevpoint=Vector2D(self.kub.get_pos())
        nextpoint=Vector2D(self.kub.get_pos())
        for gf in generatingfunction:
            prevpoint=nextpoint
            self.kub,target_point = gf
            nextpoint=self.kub.get_pos()
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            if not vicinity_points(prevpoint,nextpoint,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                self.kub.move(0,0)
                self.kub.execute()
                _GoToPoint_.init(self.kub, self.kub.get_pos(), self.theta)
                print 'BREAKINGGGG'
                break
        
    def on_exit_down(self):
        self.kub.move(0,0)
        self.kub.execute()
        pass
     
    def on_enter_left(self):
        theta = self.kub.get_pos().theta
        self.target_point.x=self.start_point.x
        self.target_point.y=self.start_point.y
        _GoToPoint_.init(self.kub, self.target_point, theta)
        #time.sleep(5)
        pass
        
    def execute_left(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        generatingfunction = _GoToPoint_.execute(start_time,self.course_approch_thresh,True)
        #self.behavior_failed = False
        time.sleep(5)
        prevpoint=Vector2D(self.kub.get_pos())
        nextpoint=Vector2D(self.kub.get_pos())
        for gf in generatingfunction:
            prevpoint=nextpoint
            self.kub,target_point = gf
            nextpoint=self.kub.get_pos()
            # self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
            if not vicinity_points(prevpoint,nextpoint,thresh=BOT_RADIUS*2.0):
                self.behavior_failed = True
                self.kub.move(0,0)
                self.kub.execute()
                _GoToPoint_.init(self.kub, self.kub.get_pos(), self.theta)
                print 'BREAKINGGGG'
                break
        
    def on_exit_left(self):
        self.kub.move(0,0)
        self.kub.execute()
        pass

     




