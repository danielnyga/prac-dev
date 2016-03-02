# -*- coding: utf-8 -*-

import traceback
import ctypes
import multiprocessing as mp
from flask import request, jsonify
from StringIO import StringIO
from flask.globals import session
from threading import Thread
import json
import logging
from prac.core.inference import PRACInference
from prac.pracutils.ActioncoreDescriptionHandler import \
    ActioncoreDescriptionHandler
from pracmln.mln.util import out
from pracmln.praclog import logger
from pracweb.gui.pages.buffer import RequestBuffer
from pracweb.gui.pages.utils import ensure_prac_session
from pracmln import Database
from pracweb.gui.app import pracApp
from prac.core.wordnet import WordNet
from prac.core.wordnet_online import WordNet as AcatWordnet

import sys
import os

import rospy
import std_msgs.msg

# imports the service
from prac2cram.srv import Prac2Cram
#ActionCore[] action_cores # each action core contains a list of action roles
#string plan  # plan name according to the last (executable) action core

# import the messages
from prac2cram.msg import ActionCore, ActionRole

log = logger(__name__)
wn = WordNet(concepts=None)
awn = AcatWordnet(concepts=None)

@pracApp.app.route('/prac/_execute_plan', methods=['POST', 'GET'])
def execute_plan():
    pracsession = ensure_prac_session(session)
    #pracsession.ĺog.info('execute_plan called') # warum error??
    method = request.method
    if method == 'POST':
        data = json.loads(request.get_data())
    else:
        data = None
    pracsession.infbuffer = RequestBuffer()
    t = Thread(target=_execute_plan, args=(pracsession, 180, method, data))
    t.start()
    pracsession.infbuffer.waitformsg()
    pracsession.log.info('received msg: %s' %pracsession.infbuffer.content) #TODO ist noch falsch
    return jsonify(pracsession.infbuffer.content)


def _execute_plan(pracsession, timeout, method, data):

    #pracsession.log.info('executing plan')
    pracsession.log.info(data)

    plan = data['plan']

    # TODO fill in real actionroles from inference
    core1 = ActionCore()
    core1.action_core_name = 'Starting'
    core1.action_roles = [ActionRole(role_name='obj_to_be_started', role_value='centrifuge.n.01')]
    action_cores = [core1]

    try:
        rospy.wait_for_service('prac2cram', timeout=5)

    except rospy.ROSException, e:
        pracsession.log.error(e)
        pracsession.infbuffer.setmsg({'status': -1, 'message': 'Timeout exceeded. Please start the ROS service "Prac2Cram" and try again.'})

    try:
        # create a handle to the service
        prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)
        resp = prac2cram(action_cores, plan)
        pracsession.log.info('Response: %s' %resp)
        if resp.status: # if error
            message = 'The CRAM service request failed!'
        else:
            message = 'CRAM service request successful!'
        pracsession.infbuffer.setmsg({'status': resp.status, 'message': message})

    except rospy.ServiceException, e:
        print "Service call failed with the following error: %s" %e
        return
