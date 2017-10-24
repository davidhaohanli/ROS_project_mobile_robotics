wordList=['five','one','two','three','four','five','three']
with open("/home/turtlebot/BJ/filename.txt", 'w') as f:
    for s in wordList:
        f.write(s + '\n')
