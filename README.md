# turtle_sim_xbox_joystick
Control a virtual robot using the xbox joystick


**Starting your terminal**
  
On your computer home screen hit these buttons at the same time. This will open a new terminal for you. 

     ctrl alt t
    
Once you have your terminal open, you must source your workspace and let it know you will be running ROS.  


    source /opt/ros/noetic/setup.bash
  
    roscore
  
  
  
**Creating a new Catkin Workspace**

_In a new terminal:_

    source /opt/ros/noetic/setup.bash
  
    mkdir -p ~/catkin_ws_turtlemove/src
  
    cd ~/catkin_ws_turtlemove/
  
    catkin_make
  
  
  
**Installing Turtlesim**

    cd ~/catkin_ws_turtlemove/src

    sudo apt-get install ros-$(rosversion -d)-turtlesim

**Running Turtlesim with keyboard control**

_In a new terminal:_

    source /opt/ros/noetic/setup.bash

    cd ~/catkin_ws_turtlemove/src

    rosrun turtlesim turtlesim_node
You should see a Virtual Turtle pop up on your screen, such as this one. However you will not be able to move the turtle using the arrow keys until the next step.


![turtlesim](https://user-images.githubusercontent.com/103153240/163054417-7287e1a9-9d29-4257-bc92-4f7d570f3009.png)



_In a new terminal:_

    source /opt/ros/noetic/setup.bash

    rosrun turtlesim turtle_teleop_key
    
You should now be able to control the Virtual Turtle with your keyboard arrow keys.



**Controlling the Turtle with a joystick**



**Installing Joy**

_In a new terminal:_

    Source /opt/ros/noetic/setup.bash
    
    cd ~/catkin_ws_turtlemove/src
    
    sudo apt-get install ros-noetic-joy
    
    
 **Configuring the Joystick**
 
 Make sure your controller is plugged into the computer.

    cd ~/catkin_ws_turtlemove/src
    
    ls /dev/input/    %This is a lower case L not an I or 1.
    
    
Here you will see a list, you will find your controller under the name: js().
For example my controller was js0.


![joystick_input](https://user-images.githubusercontent.com/103153240/163067911-094785ea-ecbe-4387-b8c9-4a715b47d630.png)

   
   
    sudo jstest /dev/input/js()    %Make sure you input your controller name for js() example: js0
    
  Once you run this test you can move around your controller and you will see the data show up in your terminal.
  
  ![data_from_joystick](https://user-images.githubusercontent.com/103153240/163243264-b71a7f1f-68b2-4ae9-93a5-994b196e3cd6.png)
  
      ls -l /dev/input/js()   %This is a lower case L not an I or 1.
      
   If your joystick is configured properly you will see(Or something similar):
   
   ![sudo_chmod_joystick](https://user-images.githubusercontent.com/103153240/163245523-9824e7f0-bfc2-453e-b3d9-5bc5f0d648c0.png)
   
   If your joystick is NOT configured properly you will see(Or something similar):
   
   ![sudo_chmod_joystick_notconfig](https://user-images.githubusercontent.com/103153240/163245576-d5fc5a68-34f7-460f-a13c-303c1dfa49d3.png)
   
   If your joystick isn't configured run this line of code:
   
      sudo chmod a+rw /dev/input/js()

_In a new terminal:_

    source /opt/ros/noetic/setup.bash

    rosparam set joy_node/dev "/dev/input/js()"
    
    rosrun joy joy_node
    
 _In a new terminal:_
 
    source /opt/ros/noetic/setup.bash
    
    rostopic echo joy
    
 
 **Writting a Teleoperation Node for the Joystick**
   
 **Create a Catkin Package**
 
 _In a new terminal:_
 
    source /opt/ros/noetic/setup.bash
 
     cd ~/catkin_ws_turtlemove/src
 
     catkin_create_pkg  learning_joy rospy turtlesim joy
 
     cd ~/catkin_ws_turtlemove/
 
     catkin_make
 
 **Creating launch and script folders**
 
 _In a new terminal:_
 
    source /opt/ros/noetic/setup.bash
    
    cd ~/catkin_ws_turtlemove/src
    
    mkdir launch
    
    mkdir scripts
    
 **Creating the Launch and Script files**
 
    cd ~/catkin_ws_turtlemove/src

    cd launch

    touch turtle_launch_file.launch

    gedit turtle_launch_file.launch

Paste the following code into your launch file (Make sure the js0 on line #8 matches your controller name):
 
    <launch>
     <!-- Turtlesim Node-->
      <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node"/>

     <!-- joy node -->
      <node respawn="true" pkg="joy"
            type="joy_node" name="turtle_launch_file" >
        <param name="dev" type="string" value="/dev/input/js0" />
        <param name="deadzone" value="0.12" />
      </node>

     <!-- Axes -->
      <param name="axis_linear" value="1" type="int"/>
      <param name="axis_angular" value="0" type="int"/>
      <param name="scale_linear" value="2" type="double"/>
      <param name="scale_angular" value="2" type="double"/>
      <node pkg="learning_joy" type="mapping.py" name="teleop"/>
    </launch>

Go back to your terminal and hit Ctrl C.

    cd ~/catkin_ws_turtlemove/src
    
    cd learning_joy
    
    touch mapping.py
    
    gedit mapping.py
    
 Insert the following code into your map.py file:
 
    #!/usr/bin/python3


    import rospy

    from geometry_msgs.msg import Twist

    from sensor_msgs.msg import Joy


    def initial(data):
      move = Twist()
      move.linear.x = 2.5 * data.axes[1]
      move.angular.z = 2.5 * data.axes[0]

      pub.publish(move)

    def movement():
      global pub
      pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size =1)
      rospy.init_node('Joy_Movement')
      rospy.Subscriber("joy", Joy, initial)

      rospy.spin()

    if __name__ == '__main__':
      movement()
    
  Go back to your terminal and hit Ctrl C
  
  
  **Launching the Turtlebot with Teleoperation**
  _In a new terminal:_
  
  NOTE: On the second line of code make sure to change "labuser" to whatever your profile is on your computer. (Example: source /home/johndoe/......)
      
      
      source /opt/ros/noetic/setup.bash
      
      source /home/labuser/catkin_ws_turtlemove/devel/setup.bash
      
      cd ~/catkin_ws_turtlemove/src
      
      cd launch
      
      roslaunch turtle_launch_file.launch
      
  You should now be able to teleoperate your virtual turtle using the joystick.
  
