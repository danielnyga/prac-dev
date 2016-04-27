# -*- coding: utf-8 -*-

from flask import request, jsonify
from flask.globals import session
from threading import Thread
import json
from pracmln.praclog import logger
from pracweb.gui.pages.buffer import RequestBuffer
from pracweb.gui.pages.utils import ensure_prac_session
from pracweb.gui.app import pracApp
from prac.core.wordnet import WordNet
from prac.core.wordnet_online import WordNet as AcatWordnet

from requests import ConnectionError

try:
    import rospy
    # imports the service
    from prac2cram.srv import Prac2Cram
    # ActionCore[] action_cores # each action core has a name and a list of action roles
    # import the ROS messages
    from prac2cram.msg import ActionCore, ActionRole
    pracApp.app.config['rospy'] = True

except ImportError:
    pracApp.app.config['rospy'] = False
    from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
    from tinyrpc.transports.http import HttpPostClientTransport
    from tinyrpc import RPCClient


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

    if not hasattr(pracsession, 'actioncores'):
        pracsession.infbuffer.setmsg({'status': -1, 'message': 'No PRAC model available. Please repeat the inference.'})        

    pracsession.log.info('Sending action cores to CRAM...') 

    if pracApp.app.config['rospy']:
        pracsession.log.info('ROS is installed. Will try to call ROS Service "Prac2Cram" directly.')

        ros_actioncores = []
        for ac in pracsession.actioncores:
            ros_actioncores.append(ActionCore(ac['action_core_name'], [ActionRole(**r) for r in ac['action_roles']]))
        pracsession.log.info('ActionCores: %s' %ros_actioncores) 

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
            
            if resp:
                pracsession.log.info('Response: %s' %resp)
                if resp.status: # if error
                    message = 'The CRAM service request failed! ' + resp.message
                else:
                    message = 'CRAM service request successful!' 
                pracsession.infbuffer.setmsg({'status': resp.status, 'message': message})
            else:
                pracsession.infbuffer.setmsg({'status': -1, 'message': 'Error: Got no response from Service call.'})


        except rospy.ServiceException, e:
            message = "Service call failed with the following error: {}".format(e.message)
            log.error(message)
            pracsession.infbuffer.setmsg({'status': -1, 'message': message})
            return
    else:
        pracsession.log.info('No ROS installation found. Will call "Prac2Cram" via RPC.')

        rpc_actioncores = pracsession.actioncores

        pracsession.log.info('ActionCores: %s' %rpc_actioncores) 

        print rpc_actioncores

        RPC_HOST = pracApp.app.config['RPC_HOST'] 
        RPC_PORT = pracApp.app.config['RPC_PORT'] 

        rpc_client = RPCClient(
            JSONRPCProtocol(),
            HttpPostClientTransport(RPC_HOST + ':' + RPC_PORT + '/')
        )
        remote_server = rpc_client.get_proxy()

        try:
            resp = remote_server.prac2cram_client(rpc_actioncores)
            if resp: # resp is now a dictionary!
                if resp['status']: # if error
                    message = 'The CRAM service request failed! ' + resp['message']
                else:
                    message = 'CRAM service request successful!' 
                pracsession.infbuffer.setmsg({'status': resp['status'], 'message': message})
            else:
                pracsession.infbuffer.setmsg({'status': -1, 'message': 'Error: Got no response from Service call.'})

        except ConnectionError:
            pracsession.infbuffer.setmsg({'status': -1, 'message': 'Connection Error: Please check RPC Service`s error message.'})

        pracsession.log.info("RPC Server answered: %s" %resp)

        


### Example input:
### [{'action_roles': [{'role_name': 'obj_to_be_started', 'role_value': 'centrifuge.n.01'}, {'role_name': 'action_verb', 'role_value': 'begin.v.03'}], 
### 'action_core_name': 'Starting'}, {'action_roles': [{'role_name': 'action_verb', 'role_value': 'begin.v.03'}], 'action_core_name': 'TurningOnElectricalDevice'}]


