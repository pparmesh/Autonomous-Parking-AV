'''
Script to Control the vehicles using the LQR Controller. 
[No Rendering]
[This script interfaces with the Carla server, receives the states of the actor vehicles and sends the control commands]
Author: Ashish Roongta
SafeAI Lab
Carnegie Mellon University
Copyright @ SafeAI lab-Carnegie Mellon University
'''

from __future__ import print_function

import glob
import os
import sys
try:
    sys.path.append(glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random


# -------importing the LQR Controller-------------
from LQR_Controller import *
from spawn_parked import *

class controller1():
    def __init__(self):
        self.no_rendering=False
        self.world=None
        self.map=None
        self.ref_traj=None
        self.contoller=None
        self.spawn_point=None
        self.player=None

    def _render(self,world):
        '''
        Function to disable rendering
        '''
        settings=world.get_settings()
        settings.no_rendering_mode=True
        world.apply_settings(settings)

    def actor_spawn(self):
        '''
        Function to spawn the actor
        '''
        spawn_point1=random.choice(self.map.get_spawn_points())  # Randomly choosing a spawn point by querying from the map
        
        spawn_point1.location.x=self.ref_traj[0,0]
        spawn_point1.location.y=self.ref_traj[0,1]
        spawn_point1.location.z=5
        spawn_point1.rotation.pitch=0.00193294
        spawn_point1.rotation.yaw=self.ref_traj[0,2]
        spawn_point1.rotation.roll=-0.00494385

        self.spawn_point=spawn_point1

        blueprint_library=self.world.get_blueprint_library()
        bp=random.choice(blueprint_library.filter('vehicle.bmw.grandtourer'))
        color=random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color',color)
        
        self.player=self.world.spawn_actor(bp,self.spawn_point)


        
    def game_loop(self,args):
        '''
        Function to execute the game loop.
        Instantiates the carla world, connects to the client
        '''
        try:
            client=carla.Client(args.host,args.port)
            client.set_timeout(2.0)

            self.world=client.get_world()  # recieveing the world info from the simulator
            self.map=self.world.get_map()
            if self.no_rendering:  # Disabling the rendering 
                self._render(self.world)
            
            empty_lot = np.arange(109)  #[59, 48, 33, 38, 39,44, 70, 10, 11, 12, 15]
            spawn_parkV(carla, client, empty_lot)

            self.ref_traj=np.load('waypoints.npy')
            self.actor_spawn()  #spawning the actor
            self.contoller=Controller2D(self.player,self.ref_traj,carla)
                

            while True:
            #LQR Control
                self.contoller.update_values()
                if self.contoller.update_controls():
                    print('Completed........!!')
                    self.player.destroy()
                    break
        finally:
            # if (self.world and self.world.recording_enabled):
            #     self.client.stop_recorder()
            actors = self.world.get_actors()
            for actor in actors:
                actor.destroy()

            if self.player is not None:
                self.player.destroy()
            

def main():
    argparser=argparse.ArgumentParser(description='CARLA Control Client')
    argparser.add_argument('-v', '--verbose',action='store_true',dest='debug',help='print debug information')
    argparser.add_argument('--host',metavar='H',default='127.0.0.1',help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port',metavar='P',default=2000,type=int,help='TCP port to listen to (default: 2000)')
    argparser.add_argument('-a', '--autopilot',action='store_true',help='enable autopilot')
    argparser.add_argument('--res',metavar='WIDTHxHEIGHT',default='1280x720',help='window resolution (default: 1280x720)')
    argparser.add_argument('--filter',metavar='PATTERN',default='vehicle.*',help='actor filter (default: "vehicle.*")')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    # log_level = logging.DEBUG if args.debug else logging.INFO
    # logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    # logging.info('listening to server %s:%s', args.host, args.port)

    # print(__doc__)

    try:

        cnlr=controller1()
        cnlr.game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__=='__main__':
    main()