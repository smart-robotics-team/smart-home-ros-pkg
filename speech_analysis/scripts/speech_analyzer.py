#!/usr/bin/env python

#!/usr/bin/env python

#    Copyright (C) 2016  Kriegel Joffrey
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
#import unicodedata
from std_msgs.msg import String
from std_msgs.msg import Empty
from jsk_gui_msgs.msg import VoiceMessage
from unidecode import unidecode

# Node example class.
class SpeechAnalyzer():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
	
        # Create a publisher for our custom message.
        self.pub_milight2ON = rospy.Publisher('/MILIGHT/light2ON', Empty, queue_size=10)
        self.pub_milight2OFF = rospy.Publisher('/MILIGHT/light2OFF', Empty, queue_size=10)
        self.pub_milight1ON = rospy.Publisher('/MILIGHT/light1ON', Empty, queue_size=10)
        self.pub_milight1OFF = rospy.Publisher('/MILIGHT/light1OFF', Empty, queue_size=10)
        self.pub_showerON = rospy.Publisher('/HOME/showerHeatON', Empty, queue_size=10)
        self.pub_showerOFF = rospy.Publisher('/HOME/showerHeatOFF', Empty, queue_size=10)
	rospy.Subscriber("/Tablet/voice", VoiceMessage, self.callback)

        # Main while loop.
        while not rospy.is_shutdown():
	    # Sleep for a while before publishing new messages. Division is so rate != period.
            rospy.sleep(0.1)

    def callback(self, data):

        # actions
        allumer_words=['allume','allumer']
        eteindre_words=['eteins','eteindre', 'eteint']

        # sujet
        lumiere_words=['lumiere','lumieres', 'lampe', 'lampes']
        prise_words=['prise','prises']

        # lieu
        salon_words=['salon','salons']
        cuisine_words=['cuisine','cuisines']
        douche_words=['douche','douches']
        chambre_words=['chambre','chambres', 'lit', 'lits']


        sentence = data.texts[0].lower()
        s1 = unicode(sentence,'utf-8')
        sentence = unidecode(s1)
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", sentence)

        words= sentence.split(' ')


        for word in words:
            if word in allumer_words:
                for word2 in words:
                    if word2 in lumiere_words:
                        for word3 in words:
                            if word3 in salon_words:
                                # allumer lumiere du salon
                                self.pub_milight2ON.publish(Empty())
                            elif word3 in chambre_words:
                                # allumer lumiere de la chambre
                                self.pub_milight1ON.publish(Empty())
                    elif word2 in prise_words:
                        for word3 in words:
                            if word3 in salon_words:
                                # allumer prise du salon
                                self.pub_showerON.publish(Empty())
                            elif word3 in douche_words:
                                # allumer prise de la douche
                                self.pub_showerON.publish(Empty())
            elif word in eteindre_words:
                for word2 in words:
                    if word2 in lumiere_words:
                        for word3 in words:
                            if word3 in salon_words:
                                # eteindre lumiere du salon
                                self.pub_milight2OFF.publish(Empty())
                            elif word3 in chambre_words:
                                # eteindre lumiere de la chambre
                                self.pub_milight1OFF.publish(Empty())
                    elif word2 in prise_words:
                        for word3 in words:
                            if word3 in salon_words:
                                # eteindre prise du salon
                                self.pub_showerOFF.publish(Empty())
                            elif word3 in douche_words:
                                # eteindre prise de la douche
                                self.pub_showerOFF.publish(Empty())

        #self.pub.publish("You just say : " + data.data)
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", data.data, self.channel)


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('speech_analyzer')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        sa = SpeechAnalyzer()
    except rospy.ROSInterruptException: pass


