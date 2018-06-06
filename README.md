# In Real
## idle with original controller
roslaunch idle_diabolo.launch
roseus juggle.l
  idle t t t
  
### demo
roslaunch demo_idle_diabolo.launch

## idle with MPC
roslaunch idle_diabolo.launch
roseus juggle.l
  idle t t t :diabolo-system t

## toss
roslaunch toss_diabolo.launch
roseus juggle.l
  toss

# In Simulation
## idle with original controller

## idle with MPC
idle_diabolo_simulate.launch
  roslaunch pr2_description upload_pr2.launch
  roseus juggle.l
    idle nil t t :simulate t :diabolo-system t ;; subscribe diabolo-system-input and send *ri*
    (idle nil t t :simulate t ;; subscribe state and calc input and send *ri*)
  rosrun jsk_diaoblo_pr2 publish_diabolo_model   ;; publish diabolo dae model
  roslaunch jsk_diabolo_pr2 robot_state_publisher   ;; translate joint_states to tf for RViz
  rosrun rviz rviz -d ~/.rviz/idle_simulate.rviz
python diabolo_system.py -m ../log/diabolo_system/goodmodel_612_0.h5 -a 1 ;; simulate(calc and publish next state) and optimize_input(calc and publish next input) 
