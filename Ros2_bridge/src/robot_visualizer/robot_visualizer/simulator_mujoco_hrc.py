'''
this file is just for debugging, can be deleted 
'''



import numpy as np
import mujoco
import mujoco_viewer



import pkg_resources

'''
NOTE::
Use mj_forward when you need to update the model state without advancing time.
Use mj_step to advance the simulation and integrate the dynamics forward in time.
So, when change the qpos or qvel, we need to run mj_forward to update the positions, orientations, velocities, and accelerations of all bodies in system
'''
'''
NOTE:: pip install -e . this will install by symlink !!
the package itself will not be copied into lib/python3.10/site-packages/. 
Instead, you will find a symlink or a .egg-link file in the site-packages ::

mujoco-sim-hrc.egg-link
'''

def rotation_matrix_from_vectors(vector1, vector2):
    # Normalize the vectors
    v1 = vector1 / np.linalg.norm(vector1)
    v2 = vector2 / np.linalg.norm(vector2)
    
    # Cross product and dot product
    v = np.cross(v1, v2)
    c = np.dot(v1, v2)
    
    # If vectors are already aligned, return the identity matrix
    if np.allclose(v, [0, 0, 0]):
        return np.eye(3)
    
    # Skew-symmetric cross-product matrix
    V = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])
    
    # Rotation matrix using Rodrigues' formula
    R = np.eye(3) + V + V @ V * ((1 - c) / (np.linalg.norm(v) ** 2))
    return R

class simulator_mujoco_v1():
    '''
    this is using the mujoco-python-viewer
    '''
    def __init__(self,) :
        pass

    def load_model(self,
                   path_xml,
                   ):
        # Load your model and data
        self.model = mujoco.MjModel.from_xml_path(path_xml)
        self.data = mujoco.MjData(self.model)
        

    def save_model(self,path_to_save):
        # Save the current state to an XML file
        mujoco.mj_saveLastXML(path_to_save, self.model)
        
    def create_viewer(self,):
        # Create the viewer
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
    
    
    '''
    # Example usage
    point = np.array([0.5, 0, 0.1])
    curve_points = np.array([
        [0.5, 0, 0.1],
        [0.6, 0.1, 0.2],
        [0.7, 0.2, 0.3],
    ])
    '''
    '''
    add_line do not work, no argument called dir 
    '''
    # Function to add a line segment
    def add_line(self, start, end, color=(0, 1, 0, 1), size=0.005):
        vect_z = np.array([0,0,1.0])
        vect_new = end - start

        self.viewer.add_marker(
            pos=(start+end)/2,
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            size=[size, size, np.linalg.norm(vect_new) / 2],
            # dir=end - start,
            # mat = rotation_matrix_from_vectors(vect_new,vect_z),
            mat = rotation_matrix_from_vectors(vect_z,vect_new),  # this looks good 
            rgba=color
        )

    # Function to add a curve
    def add_curve(self, points, color=(0, 1, 0, 1), size=0.005):
        for i in range(len(points) - 1):
            self.add_line( points[i], points[i + 1], color, size)

    # Function to add a box with orientation
    def add_box(self, position, size, orientation=np.eye(3), color=(0, 1, 0, 1)):
        """
        Adds a box to the MuJoCo viewer with a specified orientation.

        Parameters:
            viewer: The MuJoCo viewer object.
            position: The center position of the box (list or np.array of [x, y, z]).
            size: The half extents of the box in the x, y, and z directions (list or np.array of [x, y, z]).
            orientation: The 3x3 rotation matrix representing the orientation of the box (default is identity matrix).
            color: The color of the box in RGBA format (tuple of 4 floats).
        """
        self.viewer.add_marker(
            pos=position,                        # Center position of the box
            type=mujoco.mjtGeom.mjGEOM_BOX,      # Box geometry
            size=size,                           # Half extents of the box (x, y, z)
            rgba=color,                          # Color of the box (RGBA)
            mat=orientation.flatten().tolist()   # Flatten the matrix to a list and pass as 'mat'
        )

    # # Example usage
    # box_position = np.array([0.5, 0.5, 0.1])    # Center position of the box
    # box_size = np.array([0.1, 0.05, 0.02])      # Half extents of the box
    # # Example rotation matrix (90 degrees around the Z-axis)
    # rotation_matrix = np.array([
    #     [0, -1, 0],
    #     [1, 0, 0],
    #     [0, 0, 1]
    # ])
    # add_box(viewer, box_position, box_size, orientation=rotation_matrix)

    # Function to add a point
    def add_point(self, position, size=0.01, color=(1, 0, 0, 1)):
        self.viewer.add_marker(
            pos=position,
            label="",
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[size, size, size],
            rgba=color
        )
    # Function to add a point
    def add_points(self, points, size=0.01, color=(1, 0, 0, 1)):
        for i in range(len(points) ):
            position = points[i,:]
            
            self.viewer.add_marker(
                pos=position,
                label="",
                # label="origin",
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[size, size, size],
                rgba=color,

            )

    # # Function to add an arrow
    # def add_arrow(self, start, direction, length=0.1, shaft_radius=0.005, head_length=0.02, head_radius=0.01, color=(0, 0, 1, 1)):
    #     # Normalize the direction vector
    #     direction = direction / np.linalg.norm(direction)
        
    #     # End of the shaft (base of the head)
    #     shaft_end = start + (length - head_length) * direction
    #     head_end = start + length * direction

    #     # Add the shaft (capsule)
    #     self.viewer.add_marker(
    #         pos=(start + shaft_end) / 2,  # Position is the middle of the shaft
    #         type=mujoco.mjtGeom.mjGEOM_CAPSULE,
    #         size=[shaft_radius, shaft_radius, (length - head_length) / 2],
    #         mat=np.eye(3),  # Identity matrix for orientation
    #         dir=direction,
    #         rgba=color
    #     )

    #     # Add the head (cone)
    #     self.viewer.add_marker(
    #         pos=shaft_end,  # Base of the cone
    #         type=mujoco.mjtGeom.mjGEOM_CONE,
    #         size=[head_radius, head_radius, head_length],
    #         dir=direction,
    #         rgba=color
    #     )

    # # Example usage
    # start_position = np.array([0.0, 0.0, 0.0])
    # arrow_direction = np.array([1.0, 1.0, 0.0])  # Direction in 3D space
    # add_arrow(viewer, start_position, arrow_direction)
    
    # Function to add an axis arrow
    def add_axis_arrow(self, start, direction, length=0.1, color=(1, 0, 0, 1)):
        """
        Adds an arrow to represent one of the XYZ axes.

        Parameters:
            viewer: The MuJoCo viewer object.
            start: The starting position of the arrow (list or np.array of [x, y, z]).
            direction: The direction vector of the arrow (list or np.array of [x, y, z]).
            length: The length of the arrow.
            color: The color of the arrow in RGBA format (tuple of 4 floats).
        """
        # Normalize the direction vector
        direction = np.array(direction)
        direction = direction / np.linalg.norm(direction)

        # Calculate the end point of the arrow
        end = start + length * direction

        vect_z = np.array([0,0,1.0])


        # Add the arrow as a line (or you could use a capsule for a thicker arrow)
        self.viewer.add_marker(
            pos=start,
            type=mujoco.mjtGeom.mjGEOM_ARROW,  # Use an arrow geometry if available, or line/capsule
            size=[0.02, 0.02, length],       # Adjust size parameters
            rgba=color,
            # dir=direction,
            # mat = direction
            mat = rotation_matrix_from_vectors(vect_z,direction)
        )
        # print("aaa")

    def add_coordinate(self,R,t,axis_length = 0.2 ):
        origin = t
        self.add_point( origin ,
                       size=0.05,
                       color=[1, 1, 1, 1]
                       )
        
        # X-axis (Red)
        self.add_axis_arrow( origin, R[:,0], length=axis_length, color=[1, 0, 0, 0.8])
        # Y-axis (Green)
        self.add_axis_arrow( origin, R[:,1], length=axis_length, color=[0, 1, 0, 0.8])
        # Z-axis (Blue)
        self.add_axis_arrow( origin, R[:,2], length=axis_length, color=[0, 0, 1, 0.8])


    # Function to add text
    def add_text(self, position, text, color=(1, 1, 1, 1)):
        """
        Adds text to the MuJoCo viewer.

        Parameters:
            viewer: The MuJoCo viewer object.
            position: The 3D position of the text (list or np.array of [x, y, z]).
            text: The text string to display.
            color: The color of the text in RGBA format (tuple of 4 floats).
        """
        self.viewer.add_marker(
            pos=position,                # Position of the text in 3D space
            type=mujoco.mjtGeom.mjGEOM_TEXT,  # Geometry type for text
            label=text,                  # The text to display
            size=[0.1, 0.1, 0.1],        # Size of the text marker (you can adjust this)
            rgba=color                   # Color of the text (RGBA)
        )

    def render_forward(self):
        """Update kinematics from current qpos without advancing physics, then render."""
        mujoco.mj_forward(self.model, self.data)
        self.viewer.render()

    def render(self):
        """Render the viewer (call after adding markers)."""
        self.viewer.render()

    def set_Timesteps_dt(self, dt):
        self.model.opt.timestep = dt

    def step(self):
        mujoco.mj_step(self.model, self.data)

    def update(self,):
        '''
        NOTE: the dt is defined in the .xml file!!!
        '''
        mujoco.mj_step(self.model, self.data)
        self.viewer.render()


def main():
    pass

if __name__ == "__main__":
    main()