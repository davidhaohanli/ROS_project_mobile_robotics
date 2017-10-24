TIP="100.65.104.73"
sshpass -p "turtlebot" scp  ~/BJ/*.py          turtlebot@$TIP:~/BJ
sshpass -p "turtlebot" scp  ~/BJ/*.csv          turtlebot@$TIP:~/BJ
sshpass -p "turtlebot" ssh -o StrictHostKeyChecking=no turtlebot@$TIP


