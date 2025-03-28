import numpy as np
import heapq

def CheckCondition(state, condition):
    """
    Check if the condition is satisfied in the state
    """
    if (np.sum(np.multiply(state, condition))-np.sum(np.multiply(condition, condition)))==0:
        return True
    else:
        return False

def CheckVisited(state,vertices):
    """
    Check if the state is already visited
    """
    for i in range(len(vertices)):
        if np.linalg.norm(np.subtract(state,vertices[i]))==0:
            return True
    return False

def ComputeNextState(state, effect):
    """
    Compute the next state by applying the effect
    """
    newstate=np.add(state, effect)
    return newstate


def Heuristic(state, GoalIndicesOneStep, GoalIndicesTwoStep): 
    """
    Compute the heuristic value for the state
    """
    score=0
    for idx in GoalIndicesOneStep:
        if state[idx[0]][idx[1]]==-1:
            score+=1

    for idx in GoalIndicesTwoStep:
        if state[idx[0]][idx[1]]==-1 and state[idx[0]][-1]==-1:
            score+=2
        elif state[idx[0]][idx[1]]==-1 and state[idx[0]][-1]==1:
            score+=1	
            
    return score

def InitializePreconditionsAndEffects(nrObjects, nrPredicates, Predicates, Objects):
    # TODO: Fill in the preconditions and effects for incomplete actions
    ActionPre=[]
    ActionEff=[]
    ActionDesc=[]

    ### Move to hallway
    for i in range(1,5,1):
        Precond=np.zeros([nrObjects, nrPredicates])
        Precond[0][0]=-1 #Robot not in hallway
        Precond[0][i]=1  #Robot in i-th room

        Effect=np.zeros([nrObjects, nrPredicates])
        Effect[0][0]=2.  # Robot in the hallway
        Effect[0][i]=-2. # Robot not in the i-th room
    
        ActionPre.append(Precond)
        ActionEff.append(Effect)
        ActionDesc.append("Move to InHallway from "+Predicates[i])

    ### Move to room
    for i in range(1,5,1):
        Precond=np.zeros([nrObjects, nrPredicates])
        Precond[0][0]=1  # Robot in the hallway
        Precond[0][i]=-1 # Robot not in the ith room

        Effect=np.zeros([nrObjects, nrPredicates])
        Effect[0][0]=-2. # Robot not in the hallway
        Effect[0][i]=2.  # Robot in the ith room

        ActionPre.append(Precond)
        ActionEff.append(Effect)
        ActionDesc.append("Move to "+Predicates[i]+" from InHallway")

    ### Move to Pantry 
    Precond=np.zeros([nrObjects, nrPredicates])
    # TODO: Robot in the kitchen and Robot not in the pantry
    Precond[0][1] = 1
    Precond[0][5] = -1

    Effect=np.zeros([nrObjects, nrPredicates])
    # TODO: Move robot from the kitchen to the pantry (remove from kitchen and add to pantry)
    Effect[0][1] = -2 
    Effect[0][5] = 2

    ActionPre.append(Precond)
    ActionEff.append(Effect)
    ActionDesc.append("Move to Pantry from Kitchen")

    ### Move from Pantry 
    Precond=np.zeros([nrObjects, nrPredicates])
    # TODO: Robot not in the kitchen and Robot in the pantry
    Precond[0][1] = -1
    Precond[0][5] = 1

    Effect=np.zeros([nrObjects, nrPredicates])
    # TODO: Move robot from the pantry to the kitchen (remove from pantry and add to kitchen)
    Effect[0][1] = 2
    Effect[0][5] = -2
    
    ActionPre.append(Precond)
    ActionEff.append(Effect)
    ActionDesc.append("Move to Kitchen from Pantry")

    ###Cut fruit in kitchen
    for j in [1,2]:
        Precond=np.zeros([nrObjects, nrPredicates])
        # TODO: Robot in the kitchen, fruit in the kitchen, knife in the kitchen, fruit not chopped
        Precond[0][1] = 1
        Precond[j][1] = 1
        Precond[4][1] = 1
        Precond[j][6] = -1
        
        Effect=np.zeros([nrObjects, nrPredicates])
        # TODO: Fruit is chopped
        Effect[j][6] = 2

        ActionPre.append(Precond)
        ActionEff.append(Effect)
        ActionDesc.append("Cut "+Objects[j]+" in the kitchen")

    ###Pickup object
    for i in range(1,6,1):
        for j in range(1,5,1):
            Precond=np.zeros([nrObjects, nrPredicates])
            Precond[0][i]=1 #Robot in ith room
            Precond[j][i]=1 #Object j in ith room
            Precond[j][-1]=-1 #Object j not on robot

            Effect=np.zeros([nrObjects, nrPredicates])
            Effect[j][i]=-2 #Object j not in ith room
            Effect[j][-1]=2 # Object j on robot

            ActionPre.append(Precond)
            ActionEff.append(Effect)
            ActionDesc.append("Pick up "+Objects[j]+" from "+Predicates[i])
        
    ###Place object
    for i in range(1,6,1):
        for j in range(1,5,1):
            Precond=np.zeros([nrObjects, nrPredicates])
            Precond[0][i]=1 #Robot in ith room
            Precond[j][i]=-1 #Object j not in ith room
            Precond[j][-1]=1 #Object j on robot

            Effect=np.zeros([nrObjects, nrPredicates])
            Effect[j][i]=2.  #Object j in ith room
            Effect[j][-1]=-2 #Object j not on robot

            ActionPre.append(Precond)
            ActionEff.append(Effect)
            ActionDesc.append("Place "+Objects[j]+" at "+Predicates[i])
            
    return ActionPre, ActionEff, ActionDesc

def InitializeStateAndGoal(nrObjects, nrPredicates):
    InitialState=-1*np.ones([nrObjects, nrPredicates])
    InitialState[0][0]=1 # Robot is in the hallway
    InitialState[1][4]=1 # Strawberry is in the garden
    InitialState[2][5]=1 # Lemon is in the pantry
    InitialState[3][2]=1 # Paper is in the office
    InitialState[4][2]=1 # Knife is in the office
    
    GoalState=np.zeros([nrObjects, nrPredicates])
    GoalState[0][1]=1 # Robot is in the kitchen
    GoalState[1][1]=1 # Strawberry is in the kitchen
    GoalState[2][4]=1 # Lemon is in the Garden
    GoalState[1][6]=1 # Strawberry is chopped
    
    return InitialState, GoalState

np.random.seed(13)

########### Construct the Initial State and Goal State ###########
Predicates=['InHallway', 'InKitchen', 'InOffice', 'InLivingRoom', 'InGarden','InPantry','Chopped','OnRobot']
Objects=['Robot','Strawberry','Lemon', 'Paper', 'Knife'] 
nrPredicates=len(Predicates)
nrObjects=len(Objects)

ActionPre, ActionEff, ActionDesc = InitializePreconditionsAndEffects(nrObjects, nrPredicates, Predicates, Objects)
InitialState, GoalState = InitializeStateAndGoal(nrObjects, nrPredicates)

# For Heuristic
GoalIndicesOneStep=[[0,1],[1,6]]
GoalIndicesTwoStep=[[1,1],[2,4]]

########### Search ###########
vertices=[] # List of Visited States
parent=[]
action=[]
cost2come=[]
pq = [] # Use heapq to implement priority queue

# Insert the initial state
heapq.heappush(pq, (0, 0)) # (cost, vertex_id)
vertices.append(InitialState)
parent.append(0)
action.append(-1)
cost2come.append(0)

FoundPath=False
while len(pq)>0:
    # TODO: Implement Search
    # Get the element with the minimum cost
    curr_cost, curr_vertex_id = heapq.heappop(pq)
    curr_vertex = vertices[curr_vertex_id]
    
    # Check for Goal, use CheckCondition
    if CheckCondition(curr_vertex, GoalState):
        FoundPath = True
        x = curr_vertex_id
        break

    # For all actions
    for action_idx in range(len(ActionPre)):
        Precond = ActionPre[action_idx]
        Effect = ActionEff[action_idx]

        # Check if the action is applicable, UseCheckCondition
        if not CheckCondition(curr_vertex, Precond):
            continue
        
        # Get the next state, use ComputeNextState
        next_vertex = ComputeNextState(curr_vertex, Effect)
        
        # If the next state is not visited, add it to the queue. Check visited using CheckVisited
        # NOTE: 
        #   1. Append accordingly to vertices, parent, action, cost2come, and pq
        #   2. Compute the cost without heuristic for Dijkstra, and with heuristic for A*
        if not CheckVisited(next_vertex, vertices):
            vertices.append(next_vertex)
            parent.append(curr_vertex_id)
            action.append(action_idx)
            cost2come.append(cost2come[curr_vertex_id] + 1)
            
            # # Dijkstra cost
            # cost = cost2come[-1]
            
            # A* cost
            cost = cost2come[-1] + Heuristic(next_vertex, GoalIndicesOneStep, GoalIndicesTwoStep)
            
            heapq.heappush(pq, (cost, len(vertices) - 1))
    
print("Path Found: ", FoundPath)

# Extract Plan
Plan=[]
if FoundPath:
    while not x==0:
        Plan.insert(0,action[x])
        x=parent[x]
        
    # Print Plan
    print("States Explored: ", len(vertices))
    print("Plan Length: ", len(Plan))
    print()
    print("Plan:")
    for i in range(len(Plan)):
        print(ActionDesc[Plan[i]])