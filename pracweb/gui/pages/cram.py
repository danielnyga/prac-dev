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
# ActionCore[] action_cores # each action core contains a list of action roles

# import the messages
from prac2cram.msg import ActionCore, ActionRole

log = logger(__name__)
wn = WordNet(concepts=None)
awn = AcatWordnet(concepts=None)

@pracApp.app.route('/prac/_execute_plan', methods=['POST', 'GET'])
def execute_plan():
    pracsession = ensure_prac_session(session)
    #pracsession.ĺog.info('execute_plan called') # why will this throw an error here??
    method = request.method
    if method == 'POST':
        data = json.loads(request.get_data())
    else:
        data = None
    pracsession.infbuffer = RequestBuffer()
    t = Thread(target=_execute_plan, args=(pracsession, 180, method, data))
    t.start()
    pracsession.infbuffer.waitformsg()
    pracsession.log.info('received msg: %s' %pracsession.infbuffer.content)
    return jsonify(pracsession.infbuffer.content)


def _execute_plan(pracsession, timeout, method, data):

    pracsession.log.info('Sending action cores to PRAC...')
    pracsession.log.info(data)

    print pracsession.actioncores
    ros_actioncores = []

    for ac in pracsession.actioncores:
        ros_actioncores.append(ActionCore(ac['action_core_name'], [ActionRole(**r) for r in ac['action_roles']]))

    print ros_actioncores

    try:
        rospy.wait_for_service('prac2cram', timeout=5)
    # called when timeout exceeded
    except rospy.ROSException, e:
        pracsession.log.error(e)
        pracsession.infbuffer.setmsg({'status': -1, 'message': 'Timeout exceeded. Please start the ROS service "Prac2Cram" and try again.'})

    try:
        # create a handle to the ROS service
        prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)
        resp = prac2cram(ros_actioncores)
        pracsession.log.info('Response: %s' %resp)
        if resp.status: # if error
            message = 'The CRAM service request failed!'
        else:
            message = 'CRAM service request successful!'
        pracsession.infbuffer.setmsg({'status': resp.status, 'message': message})

    except rospy.ServiceException, e:
        log.error("Service call failed with the following error: {}".format(e.message))
        return
