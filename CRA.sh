TIP="100.65.104.73"
sshpass -p "turtlebot" scp -r ~/BJ/* turtlebot@$TIP:~/BJ
sshpass -p "turtlebot" ssh -o StrictHostKeyChecking=no turtlebot@$TIP


