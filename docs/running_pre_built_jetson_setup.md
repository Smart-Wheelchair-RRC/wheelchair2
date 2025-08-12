##### Alternate docker instructions for running the pre-existing setup on the wheelchair

- instructions to log in
  ```
  User : interns
  # Please ask the administrator for the password
  ```
- Run commands
  ```
  docker start wheelchair-demo-airport
  docker exec -it wheelchair-demo-airport bash
  cd wheelchair2
  source install/setup.bash
  cd src/wheelchair-navigation/shell-scripts
  
  ./navigation.tmux
  # uncomment the amcl launch command with the correct map
  # wait for the all the nodes to be brought up
  # try tele operation, if it works, all good
  # give initial pose, try to get accurate localisation 
  # give navigation goal
  ```
- To stop the commands
  ```
  # to exit tmux
  # CTRL+B, D
  
  tmux kill-server
  docker stop wheelchair-demo-airport
  ```
