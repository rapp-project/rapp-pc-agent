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


__author__ = "Konstantinos Panayiotou"
__maintainer__ = "Konstantinos Panayiotou"
__email__ = "klpanagi@gmail.com"

import pyaudio
import wave
import rospy
from os import path

from rapp_core_agent_msgs.srv import (
        Record,
        RecordResponse
        )


class AudioRecordServer:
    def __init__(self):
        print pyaudio.paInt16
        self.destDir = rospy.get_param('~default_dest_dir', '/tmp')
        self.defFilename = rospy.get_param('~default_dest_filename',        \
                                           'record.wav')
        self.defDestFile = path.join(self.destDir, self.defFilename)
        self.audioFormat = rospy.get_param('audio_format', 'wav')

        self.chunksize = rospy.get_param('~readframe_chunksize', 1024)
        self.samplerate = rospy.get_param('~samplerate', 16000)
        self.sampleFormat = rospy.get_param('~sample_precision',            \
                pyaudio.paInt16)
        self.channels = rospy.get_param('~channels', 1)
        self.srvName = rospy.get_param('~record_audio_srv_url',            \
                '/rapp_core_agent/audio_record_server/record')
        self.srv = rospy.Service(self.srvName, Record, self.handle_record_audio)
        try:
            self.pa = pyaudio.PyAudio()
        except Exception as e:
            rospy.logerr('[Audio Player Server]: %s' %e)


    def __del__(self):
        self.pa.terminate()


    def record_to_stream(self, recTime):
        rospy.loginfo('[Audio Record Server]: Start Recording for time %ssec' \
                %recTime)
        stream = self.pa.open(format=self.sampleFormat,
                              channels=self.channels,
                              rate=self.samplerate,
                              input=True,
                              frames_per_buffer=self.chunksize)

        frames = []
        for i in range(0, int(self.samplerate / self.chunksize * recTime)):
            dataframe = stream.read(self.chunksize)
            frames.append(dataframe)

        stream.stop_stream()
        stream.close()
        rospy.loginfo('[Audio Record Server]: Stoped Recording')
        return frames


    def store_rec_frames(self, audioFrames, destFile, audioFormat):
        if audioFormat == 'wav':
            try:
                wf = wave.open(destFile, 'wb')
                wf.setnchannels(self.channels)
                wf.setsampwidth(self.pa.get_sample_size(self.sampleFormat))
                wf.setframerate(self.samplerate)
                wf.writeframes(b''.join(audioFrames))
                wf.close
            except Exception as e:
                rospy.logerr(\
                        '[Audio Record Server]: Failed to write wav file\n%s' \
                        %e)
                return False
            return True


    def handle_record_audio(self, req):
        response = RecordResponse()
        recT = req.recordTime
        dataFrames = self.record_to_stream(recT)
        filename =req.filename
        if(filename == ''):
            filename = self.defFilename
        destFile = path.join(self.destDir, filename)
        if not self.store_rec_frames(dataFrames, destFile,
                                     self.audioFormat):
            response.error = "Recording failed"
            return response
        response.recFileDest = destFile
        return response

