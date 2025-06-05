# Module 8 Assignment: Path Planning with A* and RRT

## Tasks

### Task 1: Compare A* and RRT Path Planning

  - **Subtasks:**
    1. **Implement Path Planning:**

            start_point_.position.x = 5.0;
            start_point_.position.y = 5.0;
            goal_point_.position.x = 0.0;
            goal_point_.position.y = 0.0;

       - A* Algorithm

         ### How To Run?

         In *path_planning.cpp* file

         ![alt text](task1b.png)

         - Perform
            ```
            cd ~/assignment_ws
            colcon build --packages-select module_8_assignment
            source install/setup.bash
            ```
         - Run with these commands
         - Termial 1
            ```
            ros2 launch module_8_assignment task1a.launch.py 
            ```

         - Visualize in Rviz

         - Perform
            ```
            cd ~/assignment_ws
            colcon build --packages-select module_7_assignment
            source install/setup.bash
            ```
         - Terminal 2

            ```
            ros2 launch module_7_assignment task1b.launch.py
            ```
            
         ### OUTPUT

         ![alt text](task1a.png)

       - RRT Algorithm

         ### How To Run?

         In *path_planning.cpp* file

         ![alt text](task1c.png)

         - Perform
            ```
            cd ~/assignment_ws
            colcon build --packages-select module_8_assignment
            source install/setup.bash
            ```
         - Run with these commands
         - Termial 1
            ```
            ros2 launch module_8_assignment task1a.launch.py 
            ```

         - Visualize in Rviz

         - Perform
            ```
            cd ~/assignment_ws
            colcon build --packages-select module_7_assignment
            source install/setup.bash
            ```
         - Terminal 2

            ```
            ros2 launch module_7_assignment task1b.launch.py
            ```
            
         ### OUTPUT

         ![alt text](task1d.png)

    2. **Performance:**
       - Computation Time A* < RRT
       - Success Rate A* > RRT
       - Length of Path A*(~7.1m) < RRT(>7.1m) 
       - Path Smoothness A* > RRT
       - Path Efficiency A* > RRT
   
    3. **Analysis:**
      - A*: Higher memory consumption due to storing all explored nodes, but efficient with heuristics (like Manhattan or Euclidean).
      - A* is typically faster and generates shorter, more optimal paths, especially in grid-based or static environments.

      - RRT: Computational cost increases with complexity of environment or obstacle density. Random sampling can be inefficient in narrow passages unless optimized (e.g., RRT*).
      - RRT shines in high-dimensional, complex, or non-grid environments with dynamic obstacles, though it is often slower and may generate jagged paths.
      
### Task 2: Improve RRT to RRT* for Enhanced Performance

- **Objective:** Enhance the RRT algorithm by implementing RRT* (RRT Star) to improve path optimality and compete with the A* algorithm.

  - **Subtasks:**
    1. **Implement RRT*:**
       - Modify the existing RRT implementation to RRT*, which includes an optimization step to improve path quality.

    2. **Test and Compare with A*:**
       - Run the RRT* algorithm on the maze created in Assignment 6 and compare its performance with both the original RRT and A*.
       - Document any improvements in path optimality, computation time, and overall performance.

    3. **Document the Implementation:**
       - Provide detailed documentation on how RRT* was implemented, including key differences from the original RRT algorithm.
       - Explain why these improvements make RRT* more competitive with A*.

### Task 3: Explain and Write Unit Tests for RRT*

- **Objective:** Deepen your understanding of the RRT* algorithm by explaining its workings in detail and writing comprehensive unit tests to ensure its correctness.

  - **Subtasks:**
    1. **Detailed Explanation of RRT*:**
       - Write a detailed explanation of the RRT* algorithm, focusing on its key concepts such as node re-wiring, cost function optimization, and convergence properties.

    2. **Develop Unit Tests:**
       - Write unit tests for the RRT* implementation to validate its functionality.
       - Ensure the tests cover edge cases such as narrow corridors, dead-ends, and open spaces.

    3. **Run and Document Test Results:**
       - Run the unit tests and document the results, highlighting any issues found and how they were addressed.
       - Discuss the robustness of the RRT* implementation based on the test outcomes.
---
## Submission Process

1. **Create Files:**
   - Navigate to the `module_8_assignment` package.
   - Create the required files for the path planning implementations, RRT* improvements, and unit tests.

2. **Document Your Work:**
   - Create a `README.md` file in the `module_8_assignment` package.
   - Provide details about the files you created, including explanations of the code and the commands needed to run your path planning algorithms and tests.

3. **Submit Your Assignment:**
   - Push your changes to your forked repository.
   - Provide your repository link in the assignment submission text area.
   - **Note**: Ensure you press the "Start Assignment" button when you see the page (as it takes time to generate the pages).

4. **Wait for Review:**
   - Wait for the instructors to review your submission.

## Learning Outcome

By completing this assignment, you will:
- Gain a deeper understanding of path planning algorithms and their practical applications in robotics.
- Learn how to enhance the performance of path planning algorithms by implementing RRT*.
- Develop skills in writing and running unit tests to ensure the correctness and reliability of your algorithms.
