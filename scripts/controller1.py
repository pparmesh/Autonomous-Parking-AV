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
        spawn_point1.rotation.yaw=180*self.ref_traj[0,2]/np.pi
        spawn_point1.rotation.roll=-0.00494385

        self.spawn_point=spawn_point1

        blueprint_library=self.world.get_blueprint_library()

        bp=random.choice(blueprint_library.filter('vehicle.audi.tt'))
        color=random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color',color)
        
        self.player=self.world.spawn_actor(bp,self.spawn_point)

    def spawn_parkV(self, empty_lot):
        """
        Function to spawn the parked vehicles..
        """
    
        spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        blueprints=self.world.get_blueprint_library()

        batch=[]
        for i,w in enumerate(spawn_points):
            if i in empty_lot:
                continue
            blueprint = random.choice(blueprints.filter('vehicle.audi.tt'))
            color=random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color',color)
            batch.append(carla.command.SpawnActor(blueprint,w))
        return batch
        
    def game_loop(self,args):
        '''
        Function to execute the game loop.
        Instantiates the carla world, connects to the client
        '''
        try:
            client=carla.Client(args.host,args.port)
            client.set_timeout(2.0)

            self.world=client.get_world()  # recieveing the world info from the simulator
            # self.world = client.load_world("Parking1")
            # self.world.set_weather(getattr(carla.WeatherParameters, "ClearNoon"))
            self.map=self.world.get_map()
            if self.no_rendering:  # Disabling the rendering 
                self._render(self.world)
            
            try:
                empty_lot =[59, 48, 39, 44, 10,70, 81, 12, 104]
                batch = self.spawn_parkV(empty_lot)
                client.apply_batch(batch)
            except RuntimeError:
                pass 

            try:
                self.ref_traj=np.load('waypoints.npy')
                self.actor_spawn()  #spawning the actor
                self.contoller=Controller2D(self.player,self.ref_traj,carla)
            except RuntimeError:
                print("Actor not spawned")
                pass


            while True:
                try:
                    # self.world.tick()
                    #LQR Control
                    self.contoller.update_values()
                    if self.contoller.update_controls():
                        print('Completed........!!')
                        try:
                            self.player.destroy()
                            break
                        except:
                            break
                except RuntimeError:    
                    pass
        finally:
            # if (self.world and self.world.recording_enabled):
            #     self.client.stop_recorder()
            self.world - client.reload_world()
            actors = self.world.get_actors()
            for actor in actors:
                try:
                    actor.destroy()
                except ERROR:
                    continue

            try:
                if self.player is not None:
                    self.player.destroy()
            except ERROR: 
                pass

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
    except:
        pass

if __name__=='__main__':
    main()