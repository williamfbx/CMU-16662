import tkinter as tk
from PIL import Image, ImageTk
import sys

if len(sys.argv) > 1:
    image_path = sys.argv[1]
else:
    # Load the image
    image_path = "image.png"  # Change to your image file path

image = Image.open(image_path)
img_width, img_height = image.size

# Create the main window
root = tk.Tk()
root.title("Image Click Position")

# Convert the image for Tkinter
tk_image = ImageTk.PhotoImage(image)

# Create a canvas to display the image
canvas = tk.Canvas(root, width=img_width, height=img_height)
canvas.pack()

# Display the image on the canvas
canvas.create_image(0, 0, anchor=tk.NW, image=tk_image)

# Label to show coordinates
coord_label = tk.Label(root, text="Click on the image", font=("Arial", 14))
coord_label.pack()

# Function to handle mouse clicks
def on_click(event):
    # Clear previous dots
    canvas.delete("dot")
    
    # Get click coordinates
    x, y = event.x, event.y
    
    # Draw a red dot at the clicked position
    canvas.create_oval(x-6, y-6, x+6, y+6, fill="red", tags="dot")
    
    # Update label with coordinates
    coord_label.config(text=f"Clicked at: ({x}, {y})")

    # Return x and y coordinates
    return x, y

# Function to return (x,y) coordinates for pick up
def pick_up(center_point_pickup):
    # Center points for objects in the image
    center_point_1 = [439, 199]
    center_point_2 = [430, 522]
    center_point_3 = [755, 205]

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
    center_point_y_dist = center_point_1[1] - center_point_2[1]

    # Calculate distance between object and center point
    pos_x_dist = pos_1[0]-pos_2[0]
    pos_y_dist = pos_1[1]-pos_3[1]

    # Calculate distance between pickup center point and center point
    pickup_center_point_x_dist = center_point_pickup[0] - center_point_1[0]
    pickup_center_point_y_dist = center_point_pickup[1] - center_point_1[1]

    # Calculate pickup object pose
    pickup_pos_x = pos_1[0] + (pickup_center_point_x_dist*pos_x_dist/center_point_x_dist)
    pickup_pos_y = pos_1[1] + (pickup_center_point_y_dist*pos_y_dist/center_point_y_dist)

    # Return pickup object pose
    return pickup_pos_x, pickup_pos_y
    

# Bind mouse click event to the canvas
# canvas.bind("<Button-1>", on_click)

# Center point for the object to be picked up
center_point_pickup = [439, 199]
print("yes")
# Center points for the object
pickup_pos_x, pickup_pos_y = pick_up(center_point_pickup)
print(pickup_pos_x, pickup_pos_y)

# # Run the application
# root.mainloop()
