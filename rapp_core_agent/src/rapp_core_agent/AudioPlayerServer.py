#!/usr/bin/env python
# -*- coding: utf-8 -*-

##
# Copyright 2015 RAPP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#   http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Konstantinos Panayiotou
# contact: klpanagi@gmail.com

import pyaudio
import wave
from os import path
import rospy

from rapp_core_agent_ros_communications.srv import (
        AudioPlayerServerRosSrv,
        AudioPlayerServerRosSrvResponse
        )


class AudioPlayerServer:
    def __init__(self):
        self.chunksize = rospy.get_param('~readframe_chunksize', 1024)
        self.srvName = rospy.get_param('~play_audio_srv_name',              \
                '/rapp_core_agent/say')
        self.srv = rospy.Service(self.srvName, AudioPlayerServerRosSrv,     \
                self.handle_play_audio)
        self.pa = pyaudio.PyAudio()


    def __del__(self):
        self.pa.terminate()


    def play_wav_file(self, filePath):
        try:
            wf = wave.open(filePath, 'rb')
            # open stream
            stream = self.pa.open(
                    format=self.pa.get_format_from_width(wf.getsampwidth()),\
                    channels=wf.getnchannels(),                             \
                    rate=wf.getframerate(),                                 \
                    output=True)
            # read data
            data = wf.readframes(self.chunksize)

            # play stream
            while len(data) > 0:
                stream.write(data)
                data = wf.readframes(self.chunksize)
            # stop stream
            stream.stop_stream()
            stream.close()

        except Exception as e:
            rospy.logerr('[Audio Player Server]: %s' %e)
            return e


    def handle_play_audio(self, req):
        response = AudioPlayerServerRosSrvResponse()
        audioFile = path.expanduser(req.audioFile)
        filename = path.basename(audioFile)
        extension = path.splitext(filename)[1].replace(".", "")
        if(extension != "wav"):
            response.error = "Only support wav files with a .wav extension"
            return response
        if(not path.isfile(audioFile)):
            response.error = "File %s does not exist or is not a file"      \
                    %audioFile
            return response
        err = self.play_wav_file(audioFile)
        if(err not in ['', None]):
            response.error = "Failed to read audio file"
        return response


