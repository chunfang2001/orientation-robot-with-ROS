import rospy
from std_msgs.msg import String

from pydub import AudioSegment
from pydub.playback import play

from playsound import playsound

rospy.init_node('sound_node')
# song = AudioSegment.from_mp3("automode/car_horn.wav")
while not rospy.is_shutdown():
    reply = rospy.wait_for_message('playsound', String)
    if reply.data == 'play':
        # play(song)
        playsound('automode/car_horn.wav')