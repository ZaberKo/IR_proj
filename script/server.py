#!/usr/bin/env python

from pokemon.srv import Location, LocationResponse
import rospy
from math import pow, sqrt  

Pokemon_Pose = []
Pokemon_Status = []
Distance = []



# for i in range(len(Pokemon_Pose)):
#     distance_i = []
#     for j in range(len(Pokemon_Pose)):
#         i_to_j = sqrt(pow(Pokemon_Pose[i][0] - Pokemon_Pose[j][0], 2) +  
#                       pow(Pokemon_Pose[i][1] - Pokemon_Pose[j][1], 2)) 
#         distance_i.append(i_to_j)
#     Distance.append(distance_i)

def get_closest_pokemon(x, y):
    next_id = -1
    min_distance = -1
    for idx, pokemon in enumerate(Pokemon_Pose):
        if Pokemon_Status[idx] == 3:
            continue
        distance = sqrt(pow(pokemon[0] - x, 2) +  
                        pow(pokemon[1] - y, 2))
        if min_distance == -1 or distance < min_distance:
            next_id = idx
            min_distance = distance
    
    return next_id

def get_next_location(req):
    print('Request from robot: %d' % (req.robot_id))
    print('Last pokemon id: %d, Status: %d' % (req.last_id, req.status))
    print('Current Location: (%f, %f)\n' % (req.x, req.y))

    if req.last_id in range(len(Pokemon_Pose)):
        Pokemon_Status[req.last_id] = req.status

    next_id = get_closest_pokemon(req.x, req.y)
    if next_id in range(len(Pokemon_Pose)):
        Pokemon_Status[next_id] = 3
        return LocationResponse(next_id, Pokemon_Pose[next_id][0], Pokemon_Pose[next_id][1], Pokemon_Pose[next_id][3])
    
    return LocationResponse(next_id, 0, 0, 0)

def init_server():
    rospy.init_node('manage_server')
    s = rospy.Service('manage', Location, get_next_location)

    with open('./pokemon_pose.txt', 'r') as f:
        for pose in f.readlines():
            # x   y   z   yaw
            pose = pose.strip().split()
            pose = map(float, pose)
            Pokemon_Pose.append(pose)
            Pokemon_Status.append(0)

    print('Read %d pokemons.' % (len(Pokemon_Pose)))

    print("Manage Server Ready.")
    rospy.spin()

if __name__ == "__main__":
    init_server()