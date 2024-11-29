# Module 3 Assignment: URDF and Robot Creation in ROS 2

## Tasks

### Task 1: Create a Custom Transform Tree

- **Design a robotic arm with 3 DOF**
  - **Define the transform tree for the robotic arm**
  ![alt text](ArmFrames.png)
      ### How To Run?

      - Perform
         ```
         cd ~/assignment_ws
         colcon build --packages-select module_3_assignment
         source install/setup.bash
         ```
      - Run with this command
      - Termial 1
         ```
         ros2 launch module_3_assignment task1.launch.py
         ```
      
      ### Output
      ![alt text](Task1.png)
      
      - TF

      ![alt text](Task1TF.png)
  
### Task 2: Add Joints and Visual Elements

- **Enhance the robotic arm** you created earlier by adding joints:
  - **Finger Joint:** prismatic joint type.
  - **Base Joint:** continuous type.
  - **All Other Joints:** revolute joints.

      ### How To Run?

      - Perform
         ```
         cd ~/assignment_ws
         colcon build --packages-select module_3_assignment
         source install/setup.bash
         ```
      - Run with this command
      - Termial 1
         ```
         ros2 launch module_3_assignment task2.launch.py
         ```
      
      ### Output
      ![alt text](Task2a.png)
      
      - TF
      
      ![alt text](Task2aTF.png)

- **Add visualization tags** 

    ### How To Run?

        ```
    - Run with this command
    - Termial 1
        ```
        ros2 launch module_3_assignment task2b.launch.py
        ```
    
    ### Output
    
    ![alt text](Task2b.png)
      

### Task 3: Build a Mobile Manipulator

- **Integrate the robotic arm with a mobile robot platform**

    ### How To Run?

    - Perform
        ```
        cd ~/assignment_ws
        colcon build --packages-select module_3_assignment
        source install/setup.bash
        ```
    - Run with this command
    - Termial 1
        ```
        ros2 launch module_3_assignment task3a.launch.py
        ```
    
    ### Output
    ![alt text](Task3aFrames.png)
    
    ![alt text](Task3a.png)
    
    - TF

    ![alt text](Task3aTF.png)

- **Create an Ackerman Drive System:**
  - **Design a car-like robot structure** 
  ![alt text](ackermann.png)
    ### How To Run?

    - Run with these commands
    - Termial 1
        ```
        ros2 launch module_3_assignment task3b.launch.py
        ```
    - Termial 2
        ```
        ros2 launch module_3_assignment task3bgazebo.launch.py
        ```
    ### Output
    
    ![alt text](Task3b.gif)

    ![alt text](Task3bgazebo.png)
    
    - TF
    
    ![alt text](Task3bTF.png)
----