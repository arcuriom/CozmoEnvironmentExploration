from cozmo_fsm import *
import math
from wavefront import WaveFront

class SetupMap(StateNode):
    def start(self, event = None):
        super.start(event)
        rrt_instance = RRT(robot_parts=robot_parts, bbox=bbox)
        wf = WaveFront(bbox=rrt_instance.bbox)

        ### Get current world information and include it
        curr_world_map = robot.world.world_map
        x = robot.pose.position.x
        y = robot.pose.position.y
        #not sure if can get curr_objects this way
        curr_objects = curr_world_map.objects
        wf = self.put_obj_as_obstacles(curr_objects, wf)
        wf = self.fill_in_known(curr_objects, wf)
        wf = self.post_data(wf)

    def put_obj_as_obstacles(self, curr_objects, wf):
        for obj in curr_objects:
            if obj.pose.is_valid:
                xcoord = obj.pose.position.x
                ycoord = obj.pose.position.y
                wf.add_obstacle(xcoord, ycoord, obj)
                #get "line of sight" he knows then:
                m = (y-ycoord)/(x-xcoord)
                b = ycoord - a * xcoord
                start = min(x,xcoord)
                stop = max(x,xcoord)
                for i in range(start,stop):
                    line = m*i+b
                    line_width = 75
                    left = ceil(line - line_width)
                    right = floor(line+ line_width)
                    for j in range(left,right+1):
                        i1,j1 = coords_to_grid(i,j)
                        if wf.grid[i1,j1]==0:
                            # to show that we've been there and it's empty, have -1
                            wf.grid[i1,j1] = -1
        return wf

    def fill_in_known(self, curr_objects, wf):
        # set the things Cozmo can see in front of him to -1 (empty and known)
        i = y-150
        j = x-150
        while i< y+150:
            while j<x+150:
                if wf.grid[j, i]!=0:
                    break
                else:
                    wf.grid[j,i]=-1
        return wf


class Search(StateNode):
    def start(self, event = None):
        super().start(event)
        if isinstance(event, DataEvent):
            wf = event.data
            if (0 in wf.grid):
                x_start = robot.pose.position.x
                y_start = robot.pose.position.y
                start = (x_start, y_start)
                # check if there are objects visible
                objects = robot.world.world_map.objects
                (obj_x, obj_y) = self.obstacles_visible(objects, wf)
                #returns "None,None" if there aren't any
                if obj_x==None:
                    #then just go forward/towards something unseen
                    (obj_x, obj_y) = self.search_forward(wf)

                goal = (obj_x, obj_y)
                wf = self.update_map(wf, goal)

                self.post_data(goal)
            else:
                self.post_completion()

    def obstacles_visible(self, obj_list, wf):
        x_robot = robot.pose.position.x
        y_robot = robot.pose.position.y
        res = (None, None)
        for objects in obj_list:
            if object.is_visible:
                #check if we know what's behind it
                x_obj = robot.world.world_map.objects.pose.position.x
                y_obj = robot.world.world_map.objects.pose.position.y
                if x_obj>=x_robot:
                    if y_obj>=y_robot:
                        #obj is further right and forward than robot
                        check_x = x_obj+wf.inflate_size
                        check_y = y_obj+wf.inflate_size
                        if wf.grid[check_x,check_y]==0: res = (check_x, check_y)
                    else:
                        #obj is further right and further back than robot
                        check_x = x_obj+wf.inflate_size
                        check_y = y_obj-wf.inflate_size
                        if wf.grid[check_x,check_y]==0: res = (check_x, check_y)
                else:
                    if y_obj>=y_robot:
                        #obj is further left and forward than robot
                        check_x = x_obj-wf.inflate_size
                        check_y = y_obj+wf.inflate_size
                        if wf.grid[check_x,check_y]==0: res = (check_x, check_y)
                    else:
                        #obj is further left and further back than robot
                        check_x = x_obj-wf.inflate_size
                        check_y = y_obj-wf.inflate_size
                        if wf.grid[check_x,check_y]==0: res = (check_x, check_y)
            return res

        def search_forward(self, wf):
            x_robot = robot.pose.position.x
            y_robot = robot.pose.position.y
            goal = (x_robot+500, y_robot+500)
            return goal

        def update_map(self, wf, goal):
            #change values between current start and goal to be -1 (empty and known)
            x_robot = robot.pose.position.x
            y_robot = robot.pose.position.y
            i = min(x_robot, goal[0])
            j = min(y_robot, goal[1])
            max_i = max(x_robot, goal[0])
            max_j = max(y_robot, goal[1])
            while i< max_i:
                while j<max_j:

                    if wf.grid[i,j]!=0:
                        continue
                    else:
                        wf.grid[i,j]=-1
            return wf

class EnvironmentExploration(StateMachineProgram):
    def setup(self):
        """
            SearchStep: Search()
            StateNode() =N=> SetupMap() =D=> SearchStep
            SearchStep =D=> PilotToPose() =F=> Turn(90) =C=> SearchStep
            SearchStep =C=> Say("All done!")
        """
        
        # Code generated by genfsm on Wed Apr 29 14:51:21 2020:
        
        SearchStep = Search() .set_name("SearchStep") .set_parent(self)
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        setupmap1 = SetupMap() .set_name("setupmap1") .set_parent(self)
        pilottopose1 = PilotToPose() .set_name("pilottopose1") .set_parent(self)
        turn1 = Turn(90) .set_name("turn1") .set_parent(self)
        say1 = Say("All done!") .set_name("say1") .set_parent(self)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(statenode1) .add_destinations(setupmap1)
        
        datatrans1 = DataTrans() .set_name("datatrans1")
        datatrans1 .add_sources(setupmap1) .add_destinations(SearchStep)
        
        datatrans2 = DataTrans() .set_name("datatrans2")
        datatrans2 .add_sources(SearchStep) .add_destinations(pilottopose1)
        
        failuretrans1 = FailureTrans() .set_name("failuretrans1")
        failuretrans1 .add_sources(pilottopose1) .add_destinations(turn1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(turn1) .add_destinations(SearchStep)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(SearchStep) .add_destinations(say1)
        
        return self
