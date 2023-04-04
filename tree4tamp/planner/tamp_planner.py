from random import sample
from ..tamp import *
from .tamp_planner import *
from .subgoal_sampler import *
from .motion_planner import *
import time

class TAMPTree:
    def __init__(
        self, problem:TAMPProblem, max_time=100., 
        planner="gbf", heuristic="hff", eps=0.7, use_reward=True, goal_sampling=True):
        self.problem = problem
        self.tp = TaskPlanner(
            self.problem.task,
            planner=planner,
            heuristic=heuristic,
            eps=eps)
        self.ss = SubgoalSampler(self.problem)
        self.mp = MotionPlanner(self.problem)
        self.max_time = max_time
        self.solution = None
        self.start_time = 0.
        self.use_reward = use_reward
        self.goal_sampling = goal_sampling

    def plan(self, state_init:RTNode):
        self.start_time = time.time()
        print("----Planning start----")
        self.rt = RT(state_init)
        self.art = ART(ARTNode(state_init.s))
        self.art.root.rt_node_list.append(state_init)

        state_init = RTNode(self.problem.abs_state_init, self.problem.mode_init, self.problem.q_init)
        self.task_planning_layer()
        if self.solution is not None:
            self.elapsed_time = time.time()-self.start_time
            print(f"goal: elapsed_time {self.elapsed_time}[s]")
            states, commands = self.backtrack(self.solution)
            return states, commands
        else:
            self.elapsed_time = None
            print(f"failed within {self.max_time} sec.")
            return None

    def task_planning_layer(self, k_ss=2):
        iter = 1
        while time.time() - self.start_time < self.max_time:
            pi = self.tp.sample_action_sequence(self.art)
            art_node_seq = self.tp.get_ARTnode_sequence(self.art, pi)
            r = []
            for _ in range(k_ss):
                rewards = self.subgoal_sampling_layer(pi, art_node_seq)
                if self.solution is not None: return
                r += rewards
            if self.use_reward:
                self.tp.update_tree(art_node_seq, np.mean(r))
            print(f"iter{iter}: {time.time() - self.start_time}")
            iter += 1
        
    def subgoal_sampling_layer(self, pi:List[Operator], art_node_seq:List[ARTNode], k_goal=10, sample_goal_state=True):
        if not self.goal_sampling:
            batch_atts = self.ss.sample_batch_attachments(pi)
        else:
            # goal candidate generation
            for i in range(k_goal):
                if sample_goal_state:
                    # better version
                    state_goal = self.ss.sample_goal_state_directly(art_node_seq[-1].abs_state)
                else:
                    batch_atts = self.ss.sample_batch_attachments(pi)
                    state_goal = self.ss.make_goal_candidate(self.rt.root, pi, batch_atts)
                if not self.problem.is_collision(state_goal.mode, state_goal.q): 
                    if sample_goal_state:
                        batch_atts = self.ss.sample_batch_atts_with_last_mode(pi, state_goal.mode)
                    break
                state_goal = None
            if state_goal is None: return [0.]

        #batch extension
        i, rewards = 0, []
        for i in range(len(pi)):
            a, att = pi[i], batch_atts[i]
            state = art_node_seq[i].sample_rt_node()
            if state is None: break

            s, mode, q = state.unpack()
            if not a.is_geometric_action():
                state_new = RTNode(a.apply(s), mode, q)
                #state_new.set_grasp_pose(state.grasp_pose, list(state.grasp_pose.keys())[0]) # for pregrasp planning
                state_new.set_q_grasp(state.q_grasp, robot_name)
            else:
                mode_new = mode.set_attachment(att)
                q_new, q_grasp, robot_name = self.problem.sample_transition(mode, q, mode_new) # q is a pregrasp configuration
                state_new = RTNode(a.apply(s), mode_new, q_new)
                #state_new.set_grasp_pose(grasp_pose, robot_name)
                state_new.set_q_grasp(q_grasp, robot_name)  # for pregrasp planning
            
            path, approach_traj = self.motion_planning_layer(state, state_new)
            if path is not None:
                state_new.set_approach_traj(approach_traj)
                self.rt.add_child(state, state_new, a, path)
                art_node_seq[i+1].rt_node_list.append(state_new)
            else:   rewards.append(i/len(pi))
        
        if (i == len(pi)-1) and (path is not None) and (self.problem.q_goal is not None): # this is for homing action
            state_goal = RTNode(state_new.s, state_new.mode, self.problem.q_goal)
            path, approach_traj = self.motion_planning_layer(state_new, state_goal, plan_mode_switch=False)
            if path is not None:
                self.rt.add_child(state_new, state_goal, a, path)
                self.solution = state_goal
        return rewards

    def motion_planning_layer(self, state:RTNode, state_new:RTNode, plan_mode_switch:bool=True):
        if state.q == state_new.q:  return [], []

        if state_new.q_grasp is not None:
            if self.problem.is_collision(state.mode, state_new.q_grasp):
                return None, None
        if self.problem.is_collision(state.mode, state.q) or \
            self.problem.is_collision(state.mode, state_new.q):
            return None, None

        path = self.mp.rrt_connect(state.q, state_new.q, state.mode)
        if plan_mode_switch:
            robot_name = state_new.grasp_robot_name
            robot = self.problem.robots[robot_name]
            grasp_pose = robot.forward_kinematics(state_new.q_grasp.q[robot_name])
            approach_traj = self.mp.check_approach_traj(state_new, grasp_pose)
        else:
            approach_traj = []

        if path is None or approach_traj is None:
            return None, None
        return path, approach_traj

    def backtrack(self, state:RTNode):
        commands = []
        states = [state]
        while True:
            if state.parent is None: break
            command = self.rt.get_edge(state.parent, state)
            if state.parent.approach_traj is not None:
                command.traj = state.parent.approach_traj[::-1] + command.traj
            if state.approach_traj is not None:
                command.traj = command.traj + state.approach_traj
            commands.append(command)
            states.append(state.parent)
            state = state.parent
        return states[::-1], commands[::-1]

