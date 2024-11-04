#!/usr/bin/env python3
from robbie_msg.srv import Say
#from espeak import espeak
import sys
import subprocess

import rclpy
from rclpy.node import Node


class VoiceService(Node):

    def __init__(self):
        super().__init__('voice_service')
        self.srv = self.create_service(Say, 'voice', self.voice_callback)

    def voice_callback(self, request, response):
        aa = request.sentence
        #print(aa)
        response.stat = aa
        #espeak.synth(aa)
        #response.sum = request.a + request.b
        #self.get_logger().info(aa)
        language='en'
        voice='f2'
        #subprocess.run('espeak',aa)
        subprocess.call(['espeak', '-v%s+%s' % (language, voice), aa])

        return response


def main(args=None):
    rclpy.init(args=args)

    voice_service = VoiceService()

    rclpy.spin(voice_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
