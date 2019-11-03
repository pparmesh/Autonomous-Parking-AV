"""
Script to Randomly spawn vehicles in the parking spots. 
The argument takes number of parking spots to be left vacant.
The scipt is a modified version of the Carla example script "spawn_npc.py"

Author: Ashish Roongta
Carnegie Mellon University

"""

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
import random

import numpy as np

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=5,
        type=int,
        help='number of Empty Parking Sports (default: 5)')

    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='vehicles filter (default: "vehicle.bmw*")')

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(2.0)

    try:

        world = client.get_world()
        
        

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        # Randomly choising n empty parking lots
        n=args.number_of_vehicles
        empty_lot=np.random.choice(np.arange(number_of_spawn_points),n)
        blueprints=world.get_blueprint_library()

        batch=[]
        for i,w in enumerate(spawn_points):
            if i in empty_lot:
                continue
            blueprint = random.choice(blueprints.filter('vehicle.bmw.grandtourer'))
            color=random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color',color)
            batch.append(carla.command.SpawnActor(blueprint,w))
            

        for response in client.apply_batch_sync(batch):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        world.wait_for_tick()

        print('spawned %d vehicles.' % (len(vehicles_list)))

        while True:
            world.wait_for_tick()

    finally:

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
