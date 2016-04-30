# -*- coding: utf-8 -*-

from flask import request, jsonify
from flask.globals import session
from threading import Thread
import json
from pracmln import praclog
from pracmln.praclog import logger
from pracweb.gui.pages.buffer import RequestBuffer
from pracweb.gui.pages.utils import ensure_prac_session
from pracweb.gui.app import pracApp
from prac.core.wordnet import WordNet
from prac.core.wordnet_online import WordNet as AcatWordnet

from requests import ConnectionError



log = logger(__name__)
log.setLevel(praclog.INFO)

try:
    # only uncomment this to test RPC calls on a system with ROS installed
    raise ImportError
    import rospy
    # imports the service
    from prac2cram.srv import Prac2Cram
    # ActionCore[] action_cores # each action core has a name and a list of action roles
    # import the ROS messages
    from prac2cram.msg import Task, ActionCore, ActionRole
    pracApp.app.config['rospy'] = True

except ImportError:
    pracApp.app.config['rospy'] = False
    try:
        from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
        from tinyrpc.transports.http import HttpPostClientTransport
        from tinyrpc import RPCClient
        from tinyrpc import RPCError
    except ImportError:
        log.error('Neither ROS nor tinyrpc could be imported. CRAM connection will NOT work!')



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

    # tasks is a list of dictionaries each having a key called action_core containing a list 
    if not hasattr(pracsession, 'tasks'):
        pracsession.log.error('No PRAC model available!')
        pracsession.infbuffer.setmsg({'status': -1, 'message': 'No PRAC model available. Please repeat the inference.'})        
        return 

    pracsession.log.info('Will now try to send PRAC model to CRAM...') 

    # ROS #
    if pracApp.app.config['rospy']:

        pracsession.log.info('ROS is installed. Will try to call ROS Service "Prac2Cram" directly.')
        ros_tasks = [] # every task is a list of action cores 
        for task in pracsession.tasks:
            ros_task = Task()
            ros_action_cores = []
            for action_core in task['action_cores']:
                ros_action_cores.append(ActionCore(action_core['action_core_name'], [ActionRole(**r) for r in action_core['action_roles']]))
            ros_task.action_cores = ros_action_cores
            ros_tasks.append(ros_task)

        pracsession.log.info('Tasks to sent to ROS: %s' %ros_tasks) 
        print ros_tasks

        try:
            rospy.wait_for_service('prac2cram', timeout=5) # timeout in seconds
        # called when timeout exceeded
        except rospy.ROSException, e:
            pracsession.log.error(e)
            pracsession.infbuffer.setmsg({'status': -1, 'message': 'Timeout exceeded. Please start the ROS service "Prac2Cram" and try again.'})

        try:
            # create a handle to the ROS service
            prac2cram = rospy.ServiceProxy('prac2cram', Prac2Cram)
            resp = prac2cram(ros_tasks) # resp is a ROS Message 
            
            if resp:
                pracsession.log.info("ROS Server answered: %s" %resp)
                if resp.status: # if error
                    pracsession.infbuffer.setmsg({'status': -1, 'message': 'The CRAM service request failed! ' + '\n'.join(resp.message)})
                else:
                    message = 'CRAM service request successful!'
                    if hasattr(resp, 'plan_strings'): 
                        plan_strings = resp.plan_strings
                    else:
                        plan_strings = []
                    pracsession.infbuffer.setmsg({'status': 0, 'message': message, 'plan_string': '\n'.join(plan_strings)})
            else:
                pracsession.infbuffer.setmsg({'status': -1, 'message': 'Error: Got no response from Service call.'})

        except rospy.ServiceException, e:
            message = "Service call failed with the following error: {}".format(e.message)
            log.error(message)
            pracsession.infbuffer.setmsg({'status': -1, 'message': message})
    # RPC #
    else:
        pracsession.log.info('No ROS installation found. Will call "Prac2Cram" via RPC.')

        rpc_tasks = pracsession.tasks

        RPC_HOST = pracApp.app.config['RPC_HOST'] 
        RPC_PORT = pracApp.app.config['RPC_PORT'] 

        pracsession.log.info('Will connect to RPC Server:  %s:%s/' %(RPC_HOST, RPC_PORT)) 
        rpc_client = RPCClient(
            JSONRPCProtocol(),
            HttpPostClientTransport('%s:%s/' %(RPC_HOST, RPC_PORT))
        )
        remote_server = rpc_client.get_proxy()

        try:
            resp = remote_server.prac2cram_client(rpc_tasks) 
            if resp: # resp is now a dictionary!
                pracsession.log.info("RPC Server answered: %s" %resp)
                if resp['status']: # if error
                    message = 'The CRAM service request failed! ' + '\n'.join(resp['message']) 
                    pracsession.infbuffer.setmsg({'status': -1, 'message': message})
                else:
                    message = 'CRAM service request successful!' 
                    if 'plan_strings' in resp:
                        plan_strings = resp['plan_strings']
                    else:
                        plan_strings = []
                    pracsession.infbuffer.setmsg({'status': 0, 'message': message, 'plan_string': '\n'.join(plan_strings)})


            else:
                pracsession.infbuffer.setmsg({'status': -1, 'message': 'Error: Got no response from Service call.'})

        except ConnectionError:
            pracsession.infbuffer.setmsg({'status': -1, 'message': 'Connection Error: Please check RPC Service`s error message.'})
        except RPCError:
            pracsession.infbuffer.setmsg({'status': -1, 'message': 'RPC Error: Please check RPC Service`s error message.'})


