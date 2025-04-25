def pick_up(center_point_pickup):
    # Center points for objects in the image
    center_point_1 = [439, 199]
    center_point_2 = [755, 205]
    center_point_3 = [430, 522]

    # Object pose for the objects in the image
    pos_1 = [0.45309013, 0.25281023, 0.04604863]
    pos_2 = [0.45468584, 0.39374847, 0.04901705]
    pos_3 = [0.58410627, 0.24772805, 0.04621511]
    
    # Get top left (x,y) coordinates for box
    x1, y1 = [302, 42]
    
    # Get top right (x,y) coordinates for box
    x2, y2 = [898, 46]
    
    # Get bottom left (x,y) coordinates for box
    x3, y3 = [308, 661]
    
    # Get bottom right (x,y) coordinates for box
    x4, y4 = [921, 650]

    # If object outside box return none
    if center_point_pickup[0] < x1 or center_point_pickup[1] < y1 or center_point_pickup[0] > x4 or center_point_pickup[1] > y4:
        print("object outside box")
        return None

    # Calculate distance between center points
    center_point_x_dist = center_point_1[0] - center_point_2[0]
    center_point_y_dist = center_point_1[1] - center_point_3[1]

    # Calculate distance between object and center point
    pos_x_dist = pos_1[0]-pos_3[0]
    pos_y_dist = pos_1[1]-pos_2[1]

    # Calculate distance between pickup center point and center point
    pickup_center_point_y_dist = center_point_pickup[0] - center_point_1[0]
    pickup_center_point_x_dist = center_point_pickup[1] - center_point_1[1]

    # Calculate pickup object pose
    pickup_pos_x = pos_1[0] + (pickup_center_point_x_dist*pos_x_dist/center_point_x_dist)
    pickup_pos_y = pos_1[1] + (pickup_center_point_y_dist*pos_y_dist/center_point_y_dist)


    # Return pickup object pose
    return pickup_pos_x, pickup_pos_y