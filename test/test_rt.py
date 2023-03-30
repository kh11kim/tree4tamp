from tree4tamp import *


class TAMPTree:
    def __init__(self, problem:TAMPProblem):
        self.problem = problem
        self.tp = TaskPlanner(
            self.problem.task,
            planner="wastar",
            heuristic="hff",
            eps=0.6)
        self.ss = SubgoalSampler(prob)
        self.mp = MotionPlanner(prob)
        self.solution = None

    def plan(self, state_init:RTNode):
        self.rt = RT(state_init)
        self.art = ART(ARTNode(state_init.s))
        self.art.root.rt_node_list.append(state_init)

        state_init = RTNode(self.problem.abs_state_init, prob.mode_init, prob.q_init)
        self.task_planning_layer()
        if self.solution is not None:
            states, commands = self.backtrack(self.solution)
            print("goal")

    def task_planning_layer(self, max_iter=100, k_ss=2):
        for i in range(max_iter):
            pi = self.tp.sample_action_sequence(self.art)
            art_node_seq = self.tp.get_ARTnode_sequence(self.art, pi)
            r = []
            for j in range(k_ss):
                rewards = self.subgoal_sampling_layer(pi, art_node_seq)
                if self.solution is not None: return
                r += rewards
            self.tp.update_tree(art_node_seq, np.mean(r))
        
    def subgoal_sampling_layer(self, pi:List[Operator], art_node_seq:List[ARTNode], k_goal=10):
        # goal candidate generation
        for i in range(k_goal):
            batch_atts = self.ss.sample_batch_attachments(pi)
            state_goal = self.ss.make_goal_candidate(self.rt.root, pi, batch_atts)
            if not prob.is_collision(state_goal.mode, state_goal.q): break
            state_goal = None
        if state_goal is None: return None

        #batch extension
        i, rewards = 0, []
        for i in range(len(pi)):
            a, att = pi[i], batch_atts[i]
            state = art_node_seq[i].sample_rt_node()
            if state is None: break

            s, mode, q = state.unpack()
            if not a.is_geometric_action():
                state_new = RTNode(a.apply(s), mode, q)
            else:
                mode_new = mode.set_attachment(att)
                q_new, grasp_pose = prob.sample_transition(mode, q, mode_new) # q is a pregrasp configuration
                state_new = RTNode(a.apply(s), mode_new, q_new)
            #
            path = self.motion_planning_layer(state, state_new)
            if path is not None: 
                self.rt.add_child(state, state_new, a, path)
                art_node_seq[i+1].rt_node_list.append(state_new)
            else:   rewards.append(i/len(pi))
        
        if (i == len(pi)-1) and (path is not None) and (prob.q_goal is not None): # this is for homing action
            path = self.motion_planning_layer(state_new, state_goal)
            if path is not None:
                self.rt.add_child(state_new, state_goal, a, path)
                #art_node_seq[i+1].rt_node_list.append(state_new)
                self.solution = state_goal
        return rewards

    def motion_planning_layer(self, state:RTNode, state_new:RTNode):
        return self.mp.rrt_connect(state.q, state_new.q, state.mode)

    def backtrack(self, state:RTNode):
        commands = []
        states = [state]
        while True:
            if state.parent is None:break
            commands.append(self.rt.get_edge(state.parent, state))
            states.append(state.parent)
            state = state.parent
        return states, commands

    def visualize(self, states, commands):
        pass



prob = ProblemKitchen(gui=True, num_box=3, goal_box_list="all")
tree = TAMPTree(prob)
state_init = RTNode(prob.abs_state_init, prob.mode_init, prob.q_init)
tree.plan(state_init)

# if solution is not None:
#     states, commands = backtrack(solution, rt)
#     print("end")


# for iter in range(10):
#     #task planning layer
#     pi, art_node_seq = tp.get_abstract_plan(art)

#     # subgoal sampling layer
    
    
#     #batch extension
#     i, rewards = 0, []
#     for i in range(len(pi)-1):
#         a, att = pi[i], batch_atts[i]
#         state = art_node_seq[i].sample_rt_node()
#         if state is None: break

#         s, mode, q = state.unpack()
#         if not a.is_geometric_action():
#             state_new = RTNode(a.apply(s), mode, q)
#         else:
#             mode_new = mode.set_attachment(att)
#             q_new, grasp_pose = prob.sample_transition(mode, q, mode_new) # q is a pregrasp configuration
#             state_new = RTNode(a.apply(s), mode_new, q_new)
#         #
        
#         success = path is not None

#         if success:
#             rt.add_child(state, state_new, a, path)
#             art_node_seq[i+1].rt_node_list.append(state_new)
#         else:
#             rewards.append(i/len(pi))
    
#     if i == len(pi)-1 and success and prob.q_goal is not None:
#         path = mp.rrt_connect(state_new, state_goal)
#         if path is not None:
#             rt.add_child(state_new, state_goal, "end", path, state_new.mode)
#             solution = state_goal
#             break
    

    
    
#     #update rewards
        
        

print("done")