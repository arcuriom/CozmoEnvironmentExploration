from cozmo_fsm import *
import math
import collections
from wavefront import WaveFront
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors

#set up global graph variables
global num_nodes
global nodes_across
global node_up_and_down
global top_excess
global bottom_excess
global enviro_graph
global current_node
global right
global behind
global visited
global queue
global vertex
vertex = -1
global previous_node
previous_node = None
global skipped
skipped = []
global to_revert
global just_did_obstacle
just_did_obstacle = False
global store_goal
global x_traveled
global y_traveled


class MakeGraph(StateNode):
    def start(self, event = None):
        if self.running: return
        super().start(event)
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, right, behind, visited

        #Gets user input to determine the size of the graph
        print("Please enter positive integer values in millimeters. Press enter now - If you would like to use the default environment size (1m out from cozmo on all sides), press enter twice")
        try:
            front = int(input("How far in front of cozmo does your environment extent in mm?"))
            behind = int(input("How far behind cozmo does your environment extent in mm?"))
            left = int(input("How far to the left of cozmo does your environment extent in mm?"))
            right = int(input("How far to the right cozmo does your environment extent in mm?"))
        except:
            print("You have entered an invalid value and the environment will be set to the default size")
            front = 1000
            behind = 1000
            left = 1000
            right = 1000


        #Calculate how many nodes and edges are necessary and which node cozmo is currently at
        width = left + right
        height = front + behind
        nodes_across = width//500
        nodes_up_and_down = height//500
        num_nodes = nodes_across * nodes_up_and_down

        #Calculate excess on the edge nodes - will compare against these later
        top_excess = height % 500
        left_excess = width % 500

        #Create empty graph stored as dictionary with node number as key and tuples of neighbors boolean
        enviro_graph = {}
        for i in range(num_nodes):
            neighbors = []
            if (i + 1) // nodes_across == i // nodes_across:
                neighbors.append(i + 1)
            if (i - 1 >= 0 and (i - 1) // nodes_across == i // nodes_across):
                neighbors.append(i - 1)
            if (i + nodes_across) < num_nodes:
                neighbors.append(i + nodes_across)
            if (i - nodes_across) >= 0:
                neighbors.append(i - nodes_across)

            enviro_graph[i] = neighbors
        self.post_completion()



#Get Cozmo to closest node to the right and forward
class GoToStart(StateNode):
    def start(self, event = None):
        if self.running: return
        super().start(event)
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, right, behind, queue, visited

        print('Go to Start')

        currx = robot.pose.position.x
        curry = robot.pose.position.y
        currz = robot.pose.position.z
        curr_angle = robot.pose.rotation.angle_z

        #TO DO: Find the location of the closest node in cozmo's coordinate set
        newx = currx + right - ((right//500) * 500)
        newy = curry - behind + ((behind//500) * 500)

        #TO DO: stores the number of the node that cozmo is on right now
        current_node = (num_nodes - (right//500)) - (nodes_across * (behind//500))

        queue = []
        for i in enviro_graph[current_node]:
            queue.append(i)
        visited = []
        for i in range(num_nodes):
            visited.append(0)

        if newx == None:
            newx = currx+500
            newy = curry
        destination = Pose(newx, newy, currz, angle_z=curr_angle)
        visited[current_node] = 1
        self.post_data(destination)


#Get position of next node in adapted BFS and maintain graph image
class SearchStep(StateNode):

    def start(self, event = None):
        if self.running: return
        # self.check_top()
        # self.check_left()
        super().start(event)
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, visited, vertex

        #self.update_visualizer()

        print('SearchStep')
        print(current_node)

        #Implies we have reached every node
        if not queue:
            self.post_completion()

        else:
            #Gets a new vertex from the queue
            vertex = queue.pop(0)

            for i in enviro_graph[vertex]:
                queue.append(i)

            neighbor = vertex
            print(neighbor)

            #Gets cozmo to the neighbor if it has not been visited
            if visited[neighbor] == 0:
                visited[neighbor] = 1

                currx = robot.pose.position.x
                curry = robot.pose.position.y
                currz = robot.pose.position.z
                curr_angle = robot.pose.rotation.angle_z

                if neighbor // nodes_across != current_node // nodes_across:
                    newy = curry + ((neighbor // nodes_across - current_node // nodes_across) * 500)
                else:
                    newy = curry
                if neighbor % nodes_across != current_node % nodes_across:
                    newx = currx + ((neighbor % nodes_across - current_node % nodes_across) * 500)
                else:
                    newx = currx

                current_node = neighbor
                destination = Pose(newx, newy, currz, angle_z=curr_angle)

                self.post_data(destination)

            else:
                self.post_success()

    # def check_top(self):
    #     global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, visited, queue, vertex
    #     if current_node < nodes_across:
    #         #it's at the top so there might be some part less that 500 that is "buffer"
    #         self.post_completion()
    # def check_left(self):
    #     global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, visited, queue, vertex
    #     if current_node%nodes_across == 0:
    #         #it's at the left so there might be some part less that 500 that is "buffer"
    #         self.post_completion()

    #Tentative visualizer code - neither of us have machines that can run matplotlib inside of a finite state machine
    def update_visualizer(self):
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, visited, queue, vertex
        #Create visualizer grid
        # create discrete colormap

        data = np.reshape(visited, (nodes_up_and_down, nodes_across))
        cmap = colors.ListedColormap(['red', 'green'])
        fig, ax = plt.subplots()
        pcm = ax.pcolormesh(data, vmin= 0, vmax=1, cmap=cmap)
        # draw gridlines
        ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
        ax.set_xticks(np.arange(0, nodes_across, 1))
        ax.set_yticks(np.arange(0, nodes_up_and_down, 1))
        ax.set_title('Environment Map with Visited Nodes Marked in Green')

        fig.show()

    #See if cozmo can see to everywhere within a circle with a 500mm radius (that he is at the center in)
    #If not, go to that node and then return
class ObstacleCheck(StateNode):
    def no_objects_in_front(self):
        return robot.world.visible_object_count() == 0

    def get_closest_obst(self):
        return next(robot.world.visible_objects, None)

    def check_top(self):
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, visited, queue, vertex
        if current_node < nodes_across:
            #it's at the top so there might be some part less that 500 that is "buffer"
            self.post_completion()
    def check_left(self):
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, visited, queue, vertex
        if current_node%nodes_across == 0:
            #it's at the left so there might be some part less that 500 that is "buffer"
            self.post_completion()

    def start(self, event = None):
        print('ObCheck')
        if self.running: return
        super().start(event)
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, current_node, previous_node, just_did_obstacle, x_traveled, y_traveled, visited, queue, vertex

        #check what cozmo can see
        if self.no_objects_in_front() or just_did_obstacle:
            just_did_obstacle = False
            self.post_success()

        else:
            self.check_top()
            self.check_left()
            currx = robot.pose.position.x
            curry = robot.pose.position.y
            currz = robot.pose.position.z
            curr_angle = robot.pose.rotation.angle_z

            obst = self.get_closest_obst()
            #if for some reason there aren't any visible objects, returns None
            if obst is type(None) or obst is None:
                self.post_success()
                return
            obst_x = obst.pose.position.x
            obst_y = obst.pose.position.y
            closest_to_obstacle = (2^32-1,2^32-1, 2^32-1)
            (newx, newy) = (None, None)
            for neighbor in enviro_graph[current_node]:
                if visited[neighbor] == 1:
                    continue

                if neighbor // nodes_across != current_node // nodes_across:
                    newy = curry + ((neighbor // nodes_across - current_node // nodes_across) * 500)
                    y_traveled = curry-newy
                else:
                    newy = curry
                    y_traveled = 0
                if neighbor % nodes_across != current_node % nodes_across:
                    newx = currx + ((neighbor % nodes_across - current_node % nodes_across) * 500)
                    x_traveled = newx-currx
                else:
                    newx = currx
                    x_traveled = 0

                prox_to_obst = ((newx-obst_x)**2 + (newy-obst_y)**2)**0.5
                if prox_to_obst < closest_to_obstacle[2]:
                    closest_to_obstacle = (newx, newy, prox_to_obst, neighbor)
            previous_node = current_node
            destination = Pose(closest_to_obstacle[0], closest_to_obstacle[1], currz, angle_z=curr_angle)

            if closest_to_obstacle[0] == None:
                newx = currx+500
                newy = curry+500
                x_traveled = 500
                y_traveled = 500

            destination = Pose(newx, newy, currz, angle_z=curr_angle)

            just_did_obstacle = True

            self.post_data(destination)


#Preliminary code to explore the excess areas on the side of the environment
class ExploreExcess(StateNode):
    def start(self, event=None):
        if self.running: return
        global current_node, enviro_graph, previous_node, top_excess, left_excess, x_traveled, y_traveled
        currx = robot.pose.position.x
        curry = robot.pose.position.y
        currz = robot.pose.position.z
        curr_angle = robot.pose.rotation.angle_z
        # if pos == a node: output a pose that goes to the excess
        if previous_node == None:
            previous_node = current_node
            #if it's at the left excess
            if current_node%nodes_across == 0:
                newx = currx - left_excess
                newy = curry
                x_traveled = newx-currx
                y_traveled = 0
            #else, it's at the top excess
            else:
                newx = currx
                newy = curry - top_excess
                x_traveled = 0
                y_traveled = newy-curry
            # if closest_to_obstacle[0] == None:
            #     newx = currx+500
            #     newy = curry+500
            destination = Pose(newx, newy, currz, angle_z=curr_angle)
            print('ExploreExcess.1 ?')
            self.post_data(destination)

class CheckGoalBehind(Turn):
    def start(self, event=None):
        global store_goal
        if self.running: return
        print('check behind')
        if isinstance(event, DataEvent):
            cur = event.data
            prev = robot.pose.position
            q = self.robot.world.particle_filter.pose[2]
            target_q = atan2(cur.position.y-prev.y, cur.position.x-prev.x)
            delta_q = wrap_angle(q - target_q) if target_q is not None else math.inf
            if delta_q < math.inf and  abs(delta_q) > 135*math.pi/180:
                print('turns')
                self.angle = Angle(cur.rotation.angle_z.radians + math.pi)
                new_ang = cur.rotation.angle_z.radians + math.pi
            else:
                self.angle = Angle(cur.rotation.angle_z.radians)
                new_ang = cur.rotation.angle_z.radians

            new_ang = Angle(new_ang)
            store_goal = Pose(cur.position.x, cur.position.y, cur.position.z, angle_z = new_ang)
            super().start(event)

#Pass Stored Goal Position on to BFS Node
class PassAlong(StateNode):
    def start(self, event=None):
        global store_goal
        if self.running: return
        super().start(event)
        print('pass along')
        global store_goal
        self.post_data(store_goal)

class GoToCurrent(StateNode):
    def start(self, event=None):
        print('Return to Current Node')
        global current_node, previous_node, x_traveled, y_traveled
        if self.running: return
        if previous_node!=None:
            previous_node == None
            extra_x = robot.pose.position.x
            extra_y = robot.pose.position.y
            currz = robot.pose.position.z
            curr_angle = robot.pose.rotation.angle_z

            newx = extra_x - x_traveled
            newy = extra_y - y_traveled
            if newx == None:
                newx = extra_x-500
                newy = extra_y-500

            destination = Pose(newx, newy, currz, angle_z=curr_angle)
            self.post_data(destination)

class GoTo(PilotToPose):
    def start(self, event=None):
        if self.running: return
        global num_nodes, nodes_across, nodes_up_and_down, enviro_graph, top_excess, left_excess, current_node, visited, queue, vertex
        print('Go To Pose')
        if isinstance(event, DataEvent):
            if self.running: return
            self.target_pose = event.data
            check_collides = RRTNode(x=self.target_pose.position.x, y=self.target_pose.position.y, q=self.target_pose.rotation.angle_z.radians)
            goingBack=False

            while self.robot.world.rrt.collides(check_collides):
                x = self.target_pose.position.x
                y = self.target_pose.position.y
                z = self.target_pose.position.z
                curr_angle = self.target_pose.rotation.angle_z
                if not goingBack:
                    x = x
                    y = y+500
                if goingBack or (x > nodes_across*500 or y > nodes_up_and_down*500):
                    goingBack = True
                    x = x
                    y = y-500
                self.target_pose = Pose(x,y,z, angle_z=curr_angle)
                check_collides = RRTNode(x=self.target_pose.position.x, y=self.target_pose.position.y, q=self.target_pose.rotation.angle_z.radians)

        super().start(event)

#Describes the Graph Explore state machine
class GraphExplore(StateMachineProgram):
    def setup(self):
        """
            StateNode() =N=> SetHeadAngle(0) =C=> MakeGraph() =C=> GoToStart() =D=> CheckGoalBehind() =C=> PassAlong() =D=> GoTo() =C=> DriveTurn(360, 50) =C=> CheckObstacles
    
            CheckObstacles: ObstacleCheck()
            CheckObstacles =S=> StartBFS
            CheckObstacles =D=> CheckGoalBehind() =C=> PassAlong() =D=> GoTo() =C=> BackToStart =C=> CheckObstacles
            BackToStart: GoToCurrent() =D=> CheckGoalBehind() =C=> PassAlong() =D=> GoTo()
    
            CheckObstacles =C=> ExcessAndBack =C=> CheckObstacles
            ExcessAndBack: ExploreExcess() =D=> CheckGoalBehind() =C=> PassAlong() =D=> GoTo() =C=> GoToCurrent() =D=> CheckGoalBehind() =C=> PassAlong() =D=> GoTo()
    
            StartBFS: SearchStep()
            StartBFS =C=> StateNode()
            StartBFS =S=> StartBFS
            StartBFS =D=> CheckGoalBehind() =C=> PassAlong() =D=> GoTo() =C=> DriveTurn(360, 50) =C=> CheckObstacles
    
    
        """
        
        # Code generated by genfsm on Fri May  8 13:59:07 2020:
        
        statenode1 = StateNode() .set_name("statenode1") .set_parent(self)
        setheadangle1 = SetHeadAngle(0) .set_name("setheadangle1") .set_parent(self)
        makegraph1 = MakeGraph() .set_name("makegraph1") .set_parent(self)
        gotostart1 = GoToStart() .set_name("gotostart1") .set_parent(self)
        checkgoalbehind1 = CheckGoalBehind() .set_name("checkgoalbehind1") .set_parent(self)
        passalong1 = PassAlong() .set_name("passalong1") .set_parent(self)
        goto1 = GoTo() .set_name("goto1") .set_parent(self)
        driveturn1 = DriveTurn(360, 50) .set_name("driveturn1") .set_parent(self)
        CheckObstacles = ObstacleCheck() .set_name("CheckObstacles") .set_parent(self)
        checkgoalbehind2 = CheckGoalBehind() .set_name("checkgoalbehind2") .set_parent(self)
        passalong2 = PassAlong() .set_name("passalong2") .set_parent(self)
        goto2 = GoTo() .set_name("goto2") .set_parent(self)
        BackToStart = GoToCurrent() .set_name("BackToStart") .set_parent(self)
        checkgoalbehind3 = CheckGoalBehind() .set_name("checkgoalbehind3") .set_parent(self)
        passalong3 = PassAlong() .set_name("passalong3") .set_parent(self)
        goto3 = GoTo() .set_name("goto3") .set_parent(self)
        ExcessAndBack = ExploreExcess() .set_name("ExcessAndBack") .set_parent(self)
        checkgoalbehind4 = CheckGoalBehind() .set_name("checkgoalbehind4") .set_parent(self)
        passalong4 = PassAlong() .set_name("passalong4") .set_parent(self)
        goto4 = GoTo() .set_name("goto4") .set_parent(self)
        gotocurrent1 = GoToCurrent() .set_name("gotocurrent1") .set_parent(self)
        checkgoalbehind5 = CheckGoalBehind() .set_name("checkgoalbehind5") .set_parent(self)
        passalong5 = PassAlong() .set_name("passalong5") .set_parent(self)
        goto5 = GoTo() .set_name("goto5") .set_parent(self)
        StartBFS = SearchStep() .set_name("StartBFS") .set_parent(self)
        statenode2 = StateNode() .set_name("statenode2") .set_parent(self)
        checkgoalbehind6 = CheckGoalBehind() .set_name("checkgoalbehind6") .set_parent(self)
        passalong6 = PassAlong() .set_name("passalong6") .set_parent(self)
        goto6 = GoTo() .set_name("goto6") .set_parent(self)
        driveturn2 = DriveTurn(360, 50) .set_name("driveturn2") .set_parent(self)
        
        nulltrans1 = NullTrans() .set_name("nulltrans1")
        nulltrans1 .add_sources(statenode1) .add_destinations(setheadangle1)
        
        completiontrans1 = CompletionTrans() .set_name("completiontrans1")
        completiontrans1 .add_sources(setheadangle1) .add_destinations(makegraph1)
        
        completiontrans2 = CompletionTrans() .set_name("completiontrans2")
        completiontrans2 .add_sources(makegraph1) .add_destinations(gotostart1)
        
        datatrans1 = DataTrans() .set_name("datatrans1")
        datatrans1 .add_sources(gotostart1) .add_destinations(checkgoalbehind1)
        
        completiontrans3 = CompletionTrans() .set_name("completiontrans3")
        completiontrans3 .add_sources(checkgoalbehind1) .add_destinations(passalong1)
        
        datatrans2 = DataTrans() .set_name("datatrans2")
        datatrans2 .add_sources(passalong1) .add_destinations(goto1)
        
        completiontrans4 = CompletionTrans() .set_name("completiontrans4")
        completiontrans4 .add_sources(goto1) .add_destinations(driveturn1)
        
        completiontrans5 = CompletionTrans() .set_name("completiontrans5")
        completiontrans5 .add_sources(driveturn1) .add_destinations(CheckObstacles)
        
        successtrans1 = SuccessTrans() .set_name("successtrans1")
        successtrans1 .add_sources(CheckObstacles) .add_destinations(StartBFS)
        
        datatrans3 = DataTrans() .set_name("datatrans3")
        datatrans3 .add_sources(CheckObstacles) .add_destinations(checkgoalbehind2)
        
        completiontrans6 = CompletionTrans() .set_name("completiontrans6")
        completiontrans6 .add_sources(checkgoalbehind2) .add_destinations(passalong2)
        
        datatrans4 = DataTrans() .set_name("datatrans4")
        datatrans4 .add_sources(passalong2) .add_destinations(goto2)
        
        completiontrans7 = CompletionTrans() .set_name("completiontrans7")
        completiontrans7 .add_sources(goto2) .add_destinations(BackToStart)
        
        completiontrans8 = CompletionTrans() .set_name("completiontrans8")
        completiontrans8 .add_sources(BackToStart) .add_destinations(CheckObstacles)
        
        datatrans5 = DataTrans() .set_name("datatrans5")
        datatrans5 .add_sources(BackToStart) .add_destinations(checkgoalbehind3)
        
        completiontrans9 = CompletionTrans() .set_name("completiontrans9")
        completiontrans9 .add_sources(checkgoalbehind3) .add_destinations(passalong3)
        
        datatrans6 = DataTrans() .set_name("datatrans6")
        datatrans6 .add_sources(passalong3) .add_destinations(goto3)
        
        completiontrans10 = CompletionTrans() .set_name("completiontrans10")
        completiontrans10 .add_sources(CheckObstacles) .add_destinations(ExcessAndBack)
        
        completiontrans11 = CompletionTrans() .set_name("completiontrans11")
        completiontrans11 .add_sources(ExcessAndBack) .add_destinations(CheckObstacles)
        
        datatrans7 = DataTrans() .set_name("datatrans7")
        datatrans7 .add_sources(ExcessAndBack) .add_destinations(checkgoalbehind4)
        
        completiontrans12 = CompletionTrans() .set_name("completiontrans12")
        completiontrans12 .add_sources(checkgoalbehind4) .add_destinations(passalong4)
        
        datatrans8 = DataTrans() .set_name("datatrans8")
        datatrans8 .add_sources(passalong4) .add_destinations(goto4)
        
        completiontrans13 = CompletionTrans() .set_name("completiontrans13")
        completiontrans13 .add_sources(goto4) .add_destinations(gotocurrent1)
        
        datatrans9 = DataTrans() .set_name("datatrans9")
        datatrans9 .add_sources(gotocurrent1) .add_destinations(checkgoalbehind5)
        
        completiontrans14 = CompletionTrans() .set_name("completiontrans14")
        completiontrans14 .add_sources(checkgoalbehind5) .add_destinations(passalong5)
        
        datatrans10 = DataTrans() .set_name("datatrans10")
        datatrans10 .add_sources(passalong5) .add_destinations(goto5)
        
        completiontrans15 = CompletionTrans() .set_name("completiontrans15")
        completiontrans15 .add_sources(StartBFS) .add_destinations(statenode2)
        
        successtrans2 = SuccessTrans() .set_name("successtrans2")
        successtrans2 .add_sources(StartBFS) .add_destinations(StartBFS)
        
        datatrans11 = DataTrans() .set_name("datatrans11")
        datatrans11 .add_sources(StartBFS) .add_destinations(checkgoalbehind6)
        
        completiontrans16 = CompletionTrans() .set_name("completiontrans16")
        completiontrans16 .add_sources(checkgoalbehind6) .add_destinations(passalong6)
        
        datatrans12 = DataTrans() .set_name("datatrans12")
        datatrans12 .add_sources(passalong6) .add_destinations(goto6)
        
        completiontrans17 = CompletionTrans() .set_name("completiontrans17")
        completiontrans17 .add_sources(goto6) .add_destinations(driveturn2)
        
        completiontrans18 = CompletionTrans() .set_name("completiontrans18")
        completiontrans18 .add_sources(driveturn2) .add_destinations(CheckObstacles)
        
        return self
