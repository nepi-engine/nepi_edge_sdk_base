#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_edge_sdk_base
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_base
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#

from collections import OrderedDict

import rospy
from sensor_msgs.msg import Image

import cv2 # Must come before cv_bridge on Jetson Ubuntu 20.04 for some reason!
from cv_bridge import CvBridge

from std_msgs.msg import String
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF
from nepi_ros_interfaces.msg import ImageMuxSequence, ImageMuxInput
from nepi_ros_interfaces.srv import ImageMuxSequenceQuery, ImageMuxSequenceQueryResponse

class SequentialImageMux(object):
    DEFAULT_NODE_NAME = 'sequential_image_mux'

    def __init__(self):
        # Launch the ROS node
        rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
        rospy.init_node(self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
        self.node_name = rospy.get_name().split('/')[-1]
        
        self.mux_sequences = OrderedDict() # Dictionary of dictionaries, keyed by sequence name

        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.updateToParamServer, paramsModifiedCallback=self.updateFromParamServer)
        self.updateFromParamServer() # Will initialize and setup all configured mux sequences

        rospy.Subscriber('~configure_mux_sequence', ImageMuxSequence, self.handleConfigureImageMuxSequence)
        rospy.Subscriber('~delete_mux_sequence', String, self.handleDeleteImageMuxSequece)

        rospy.Service('~mux_sequence_query', ImageMuxSequenceQuery, self.provideMuxSequences)
        
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.runMuxSequences()
            rate.sleep()

    def updateToParamServer(self):
        # Create the dictionary of dictionaries
        #mux_seqs = OrderedDict() -- set_param() can't handle ordered dictionaries
        mux_seqs = {}

        for seq_id, seq in self.mux_sequences.items():
            config_seq = {}
            config_seq['enabled'] = seq['enabled']
            config_seq['output_topic'] = seq['output_topic']
            config_seq['output_img_width_pixels'] = seq['output_img_width_pixels']
            config_seq['output_img_height_pixels'] = seq['output_img_height_pixels']
            config_seq['inputs'] = []
            for input in seq['inputs']:
                input_dict = {}
                input_dict['topic'] = input['topic']
                input_dict['min_frame_count'] = input['min_frame_count']
                input_dict['max_frame_count'] = input['max_frame_count']
                input_dict['min_duration_s'] = input['min_duration_s']
                input_dict['max_duration_s'] = input['max_duration_s']
                config_seq['inputs'].append(input_dict)
            mux_seqs[seq_id] = config_seq

        # Send the whole thing to the param server
        rospy.set_param("~mux_sequences", mux_seqs)

    def updateFromParamServer(self):
        configured_mux_sequences = rospy.get_param("~mux_sequences", {})

        for seq_id, config_seq in configured_mux_sequences.items():
            self.addUpdateMuxSequenceFromParams(seq_id, config_seq)

    def handleConfigureImageMuxSequence(self, msg):
        seq_id = msg.sequence_id
        rospy.loginfo(self.node_name + ": Adding/modifying sequence " + seq_id + " by request")

        param_seq = {}
        param_seq['enabled'] = msg.enabled
        param_seq['output_topic'] = msg.output_topic
        param_seq['output_img_width_pixels'] = msg.output_img_width_pixels
        param_seq['output_img_height_pixels'] = msg.output_img_height_pixels
        
        param_seq['inputs'] = []
        for msg_input in msg.inputs:
            input = {}
            input['topic'] = msg_input.topic
            input['min_frame_count'] = msg_input.min_frame_count
            input['max_frame_count'] = msg_input.max_frame_count
            input['min_duration_s'] = msg_input.min_duration_s
            input['max_duration_s'] = msg_input.max_duration_s

            param_seq['inputs'].append(input)

        self.addUpdateMuxSequenceFromParams(seq_id, param_seq)

    def provideMuxSequences(self, req):
        resp = ImageMuxSequenceQueryResponse()
        
        for seq_id, seq in self.mux_sequences.items():
            resp_seq = ImageMuxSequence()
            resp_seq.sequence_id = seq_id
            resp_seq.enabled = seq['enabled']
            resp_seq.output_topic = seq['output_topic']
            resp_seq.output_img_width_pixels = seq['output_img_width_pixels']
            resp_seq.output_img_height_pixels = seq['output_img_height_pixels']
            for input in seq['inputs']:
                resp_input = ImageMuxInput()
                resp_input.topic = input['topic']
                resp_input.min_frame_count = input['min_frame_count']
                resp_input.max_frame_count = input['max_frame_count']
                resp_input.min_duration_s = input['min_duration_s']
                resp_input.max_duration_s = input['max_duration_s']
                
                resp_seq.inputs.append(resp_input)
            
            resp.img_mux_sequences.append(resp_seq)

        return resp

    def handleDeleteImageMuxSequece(self, msg):
        seq_id = msg.data
        if seq_id not in self.mux_sequences:
            rospy.logwarn(self.node_name + ": Request to delete unknown sequence " + seq_id + "... ignoring")
            return
        
        rospy.loginfo(self.node_name + ": Deleting sequence " + seq_id + " by request")
        # Make sure to unsubscribe everything associated with this sequence first
        self.resetMuxSequence(seq_id, restart_if_enabled = False)

        # And then delete the entry
        del self.mux_sequences[seq_id] 

    def addUpdateMuxSequenceFromParams(self, seq_id, param_seq):
        # First validate this sequence dictionary by checking for required keys
        if ('enabled' not in param_seq) or ('output_topic' not in param_seq) or ('inputs' not in param_seq):
            rospy.logwarn(self.node_name + ": Invalid mux sequence " + seq_id + " specified... skipping")
            return
        
        total_frame_count_val = 0
        # Check if updating an existing sequence, and if so, stop it so that we can restart fresh with new params
        if seq_id in self.mux_sequences:
           total_frame_count_val = self.mux_sequences[seq_id]['total_frame_count']
           rospy.loginfo(self.node_name + ": resetting sequence " +  seq_id + " in order to update it")
           self.resetMuxSequence(seq_id, restart_if_enabled = False) 
        
        # Now set up/update an entire sequence object here
        new_seq = {}
                    
        # Initialize all params from config
        new_seq['enabled'] = param_seq['enabled']
        new_seq['output_topic'] = rospy.resolve_name(param_seq['output_topic']) # Get a complete namespace here to report back to service clients
        new_seq['output_img_width_pixels'] = param_seq['output_img_width_pixels'] if 'output_img_width_pixels' in param_seq else 0
        new_seq['output_img_height_pixels'] = param_seq['output_img_height_pixels'] if 'output_img_height_pixels' in param_seq else 0
        new_seq['total_frame_count'] = total_frame_count_val
        new_seq['inputs'] = [] # List of dicts
        for i, config_input in enumerate(param_seq['inputs']):
            # Check that all input params are defined -- these are all required
            if ('topic' not in config_input) or \
                ('min_frame_count' not in config_input) or ('max_frame_count' not in config_input) or \
                ('min_duration_s' not in config_input) or ('max_duration_s' not in config_input):
                rospy.logwarn(self.node_name + ": Invalid input definition in config file sequence " + seq_id + " (input " + str(i) + ")... skipping this input") 
                continue
            new_seq['inputs'].append(config_input)

        # Finally, write the update back to the member variable... this may overwrite an existing entry
        self.mux_sequences[seq_id] = new_seq
        # And restart it if enabled
        self.resetMuxSequence(seq_id, restart_if_enabled = True)

    def resetAllMuxSequences(self, restart_enabled_sequences):
        for seq_id in self.mux_sequences:
            self.resetMuxSequence(seq_id, restart_enabled_sequences)

    def resetMuxSequence(self, seq_id, restart_if_enabled):
        if seq_id not in self.mux_sequences:
            rospy.logwarn(self.node_name + ": cannot reset unknown mux sequence " + seq_id)
            return

        seq = self.mux_sequences[seq_id]                
        # Unregister all publications and subscriptions
        if 'output_pub' in seq and seq['output_pub'] is not None:
            seq['output_pub'].unregister()
        seq['output_pub'] = None
                
        for input in seq['inputs']:
            if 'subscriber' in input and input['subscriber'] is not None:
                input['subscriber'].unregister()
            input['subscriber'] = None            
        
        if (restart_if_enabled is True):
            # Now re-register and setup all params
            if seq['enabled'] is True:
                seq['output_pub'] = rospy.Publisher(seq['output_topic'], Image, tcp_nodelay = True, queue_size = 3)
                # And flag that this sequence hasn't started yet
                seq['curr_step'] = -1
                if 'cv_bridge' not in seq:
                    seq['cv_bridge'] = CvBridge()
                for step_index, input in enumerate(seq['inputs']):
                    input['subscriber'] = rospy.Subscriber(input['topic'], Image, self.processInputTopic, 
                                                        callback_args=[seq_id, step_index])
        
        self.mux_sequences[seq_id] = seq
                
                    
    def runMuxSequences(self):
        for seq_id, seq in self.mux_sequences.items():
            step_count = len(seq['inputs'])
            if (seq['enabled'] is False) or (step_count == 0) or ('curr_step' not in seq):
                continue

            now = rospy.Time.now()

            # Check for sequence step conditions
            take_step = False
            curr_step = seq['curr_step']
            # Sequence is transitioning from stopped
            if curr_step == -1: 
                take_step = True
            # Current step has exceeded max duration OR max frame count
            elif (now >= seq['curr_step_max_stop_time']) or (seq['curr_step_frame_count'] >= seq['curr_step_max_frame_count']): # Se
                take_step = True
            # Current step has exceeded min duration AND min frame count
            elif (now >= seq['curr_step_min_stop_time']) and (seq['curr_step_frame_count'] >= seq['curr_step_min_frame_count']):
                take_step = True

            if take_step and (step_count > 0):
                last_step_in_seq = step_count - 1
                next_step = (curr_step + 1) if (seq['curr_step'] < last_step_in_seq) else 0 # Wrap back to start as necessary

                # Step to next input in the sequence
                next_input = seq['inputs'][next_step]
                seq['curr_step'] = next_step
                # Reinitialize all counts and timers for the new step
                seq['curr_step_frame_count'] = 0
                seq['curr_step_start_time'] = now
                seq['curr_step_min_stop_time'] = now + rospy.Duration(next_input['min_duration_s'])
                seq['curr_step_max_stop_time'] = now + rospy.Duration(next_input['max_duration_s'])
                seq['curr_step_min_frame_count'] = next_input['min_frame_count']
                seq['curr_step_max_frame_count'] = next_input['max_frame_count']

                # TODO: Unsubscribe/resubscribe per seq['subscribe_lookahead']. This would reduce processing burden considerably
                # especially when lots of inputs

    def processInputTopic(self, msg, input_args):
        # First, determine if we currently care about this topic... otherwise, exit quickly
        seq_id = input_args[0]
        step_index = input_args[1]
        if (seq_id not in self.mux_sequences):
            rospy.logwarn(self.node_name + ": Got invalid sequence id " + seq_id + " in input image callback")
            return

        seq = self.mux_sequences[seq_id]
        if (seq['enabled'] is False):
            rospy.logwarn(self.node_name + " Got sequence id for disabled sequence " + seq_id + " in input image callback")
            return
        
        if seq['curr_step'] != step_index:
            return # This is typical -- just means we aren't on the associated step currently
        
        if seq['output_pub'] is None:
            rospy.logwarn_throttle(5.0, self.node_name + " Publiser for sequence " + seq_id + " is not defined... skipping image processing")
            return
        
        # Process the input image into the correct output size
        cv_bridge = seq['cv_bridge']
        cv_img = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        if (seq['output_img_width_pixels'] > 0) and (seq['output_img_height_pixels'] > 0):
            out_dim = (int(seq['output_img_width_pixels']), int(seq['output_img_height_pixels']))
            cv_img = cv2.resize(src=cv_img, dsize=out_dim, interpolation=cv2.INTER_AREA)

        # Convert back to ROS, but preserve the original image's header, except sequence counter
        ros_img = cv_bridge.cv2_to_imgmsg(cv_img, "bgr8")
        ros_img.header = msg.header
        ros_img.header.seq = seq['total_frame_count']

        # Debugging
        #rospy.logwarn("Debugging: Input hdr = " + str(msg.header) + "\nOutput hdr = " + str(ros_img.header))
        
        # Publish it -- double check that publisher is still active, since it can be closed asynchronously during image conversion
        if seq['output_pub'] is not None:
            seq['output_pub'].publish(ros_img)

            # Update frame count for this step
            seq['curr_step_frame_count'] += 1
            seq['total_frame_count'] += 1


if __name__ == '__main__':
	node = SequentialImageMux()


