import rospy
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys
#sudo apt-get install ros-indigo-audio-common
#sudo apt-get install libasound2
from Misc import bcolors
class SpeechManager():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.wordList=['Sentense:']
        self.previousWord=[]
        self.wordUniquelist=[]
        self.current_qr = String()

    def InitializeQrRecording(self):
        self.code_messageSubscriberHandle = rospy.Subscriber('/visp_auto_tracker/code_message', String,
                                                             self.update_current_qr)
    def TerminateQrRecording(self):
        self.code_messageSubscriberHandle.unregister()
    def update_current_qr(self, _current_qr):
        if _current_qr.data != "":
            self.current_qr = _current_qr.data
            self.AddWord(_current_qr.data)

    def InitializeTTS(self):
        # rospy.init_node('SpeechManager')
        # Set the default TTS voice to use
        #self.voice = rospy.get_param("~voice", "voice_don_diphone")
        # Create the sound client object
        self.soundhandle = SoundClient()
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        self.soundhandle.stopAll()
        self.soundhandle.say("Speech manager is ready.")#, self.voice)

    def AddWord(self,_word):
        print(bcolors.WARNING + "Message:" + bcolors.ENDC)
        print(_word)
        if(self.previousWord!=_word and _word!=''):
            self.wordList.append(_word)

    def RemoveRepeats(self):
        #print(self.wordList);
        self.wordUniquelist = []
        [self.wordUniquelist.append(item) for item in self.wordList if item not in self.wordUniquelist]
        with open("/home/turtlebot/BJ/filename.txt", 'w') as f:
            for s in self.wordUniquelist:
                f.write(s + '\n')
        #print(self.wordUniquelist);

    def ReadWords(self):
        print('Reading:')
        print(self.wordUniquelist);
        for word in self.wordUniquelist:
            self.soundhandle.say(word)
            rospy.sleep(3);
        '''
        catWords='';
        for word in self.wordUniquelist:
            catWords = catWords + word+' '
        print('Reading: ' + catWords)
        self.soundhandle.say(catWords)
        rospy.sleep(15);
        '''



    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down SpeechManager node...")