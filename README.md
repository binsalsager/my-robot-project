# my-robot-project
a file including the ai face recognition of our robot
To switch between your models, you only need to edit one line in your `ai_launch.py` file.

-----

## \#\# How to Switch Between Model Sets 🔄

1.  **Open the Launch File**
    Open your `ai` package's launch file in an editor:

    ```bash
    nano ~/project_ws/src/ai/launch/ai_launch.py
    ```

2.  **Find and Edit the Parameter**
    Look for the `Node` definition for your `vision_node`. You will see a `parameters` section.

      * To use **Model Set A** (the faster HoG detector), make sure the line reads:
        ```python
        parameters=[
            {'model_set': 'A'}
        ]
        ```
      * To use **Model Set B** (the more accurate CNN detector), change the `'A'` to a `'B'`:
        ```python
        parameters=[
            {'model_set': 'B'}
        ]
        ```

3.  **Save and Relaunch**

      * Save the file and exit the editor (`Ctrl + X`, `Y`, `Enter`).
      * You do **not** need to rebuild the workspace because you only changed a launch file.
      * Simply run the launch command again in your AI terminal:
        ```bash
        ros2 launch ai ai_launch.py
        ```

-----

### \#\# How to Verify

When the `vision_node` starts, look at its terminal output. It will print a confirmation message telling you which model set is active:

  * `[INFO] [vision_node]: --- Using Model Set: 'A' ---`
  * `[INFO] [vision_node]: --- Using Model Set: 'B' ---`
