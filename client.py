import json
import random
import socket
import sys
import math
import time
import uuid
from copy import copy, deepcopy
import threading

from frozendict import frozendict
import numpy
import pygame
from pygame import mixer
from pygame import Rect
from pygame.math import Vector2

import headered_socket
from helpers import get_matching_objects

def round_down(n):
    return int(math.floor(n / 20.0)) * 20

def create_update(update_type, entity_type=None, entity_id=None, data=None, json_bytes=False):
    # conditions that must be true if we are making a create update

    match update_type:
        case "create":
            if entity_type is None or data is None:
                raise Exception("Must supply entity_type and data arguments when making CREATE update")

        case "update":
            if entity_id is None or data is None:
                raise Exception("Must supply entity_id and data arguments when making UPDATE update")

        case "delete":
            if entity_id is None:
                raise Exception("Must supply entity_id argument when making DELETE update")

        case "sound":
            if data is None:
                raise Exception("Must supply data argument when making SOUND update")

    update = {
        "update_type": update_type,
        "entity_type": entity_type,
        "entity_id": entity_id,
        "data": data
    }

    if json_bytes:
        return bytes(json.dumps(update), "utf-8")

    return update

class State:
    def __init__(self, entities={}, updates=[], sounds=[]):
        self.entities = entities
        self.updates = updates
        self.sounds = sounds

class Entity:
    def __init__(self, rect, sprite_path, owner=None, visible=True, entity_id=None, velocity=Vector2(0, 0),
                 scale_res=None):

        # the one that updates this every frame
        self.owner = owner

        self.rect = rect

        self.velocity = velocity

        self.sprite_path = sprite_path

        self.sprite = pygame.image.load(sprite_path)

        self.visible = visible

        # this indicates to the game that we want to delete the object from the state
        self.delete = False

        self.entity_id = entity_id

        # we use the object id to identify objects. that way we can update existing objects
        if entity_id == None:  # if no object id is supplied we make one
            self.entity_id = str(uuid.uuid4())

        # all the functions that will be executed every frame
        self.update_funcs = [
            self.move
        ]

        self.scale_res = scale_res

        self.entity_type = "entity"

        # scale the sprite if a scale res is specified
        if scale_res and sprite_path:
            self.sprite = pygame.transform.scale(self.sprite, scale_res)

    @staticmethod
    def create_from_dict(entity_dict):

        rect = Rect(entity_dict["rect"])
        sprite_path = entity_dict["sprite_path"]
        owner = entity_dict["owner"]
        visible = entity_dict["visible"]
        entity_id = entity_dict["entity_id"]
        velocity = Vector2(entity_dict["velocity"])
        scale_res = entity_dict["scale_res"]

        new_object = Entity(rect=rect, sprite_path=sprite_path, owner=owner, visible=visible, entity_id=entity_id,
                            velocity=velocity, scale_res=scale_res)

        return new_object

    def load_update(self, update_data):

        for key in update_data.keys():

            match key:
                case "rect":
                    self.rect.x, self.rect.y, self.rect.width, self.rect.height = update_data["rect"]
                case "sprite_path":
                    self.sprite_path = update_data["sprite_path"]
                case "owner":
                    self.owner = update_data["owner"]
                case "visible":
                    self.visible = update_data["visible"]
                case "entity_id":
                    self.entity_id = update_data["entity_id"]
                case "velocity":
                    self.velocity.x, self.velocity.y = update_data["velocity"]
                case "scale_res":
                    self.scale_res = update_data["scale_res"]

    def dump_to_dict(self):

        data_dict = {}

        data_dict["rect"] = [self.rect.x, self.rect.y, self.rect.width, self.rect.height]
        data_dict["sprite_path"] = self.sprite_path
        data_dict["owner"] = self.owner
        data_dict["visible"] = self.visible
        data_dict["entity_id"] = self.entity_id
        data_dict["velocity"] = [self.velocity.x, self.velocity.y]
        data_dict["scale_res"] = self.scale_res

        return data_dict

    def detect_collisions(self, entities):

        # list of entities that collide with this object
        collisions = []

        for entity in entities.values():
            # tests if this entities rect overlaps with
            if self.rect.colliderect(entity.rect):
                collisions.append(entity)

        return collisions

    def move(self, state):
        # move the entities rect

        #print(self.velocity)

        # we dont need to move the entity if they have no velocity
        if self.velocity == Vector2(0,0):
            return

        self.rect.move_ip(self.velocity)

        update_data = {
            "rect": [self.rect.x, self.rect.y, self.rect.width, self.rect.height]
        }

        # add the update to the queue
        state.updates.append(
            create_update(update_type="update", entity_id=self.entity_id, data=update_data)
        )

        #print(self.velocity)

class Block(Entity):
    pass

class Bullet(Entity):
    def __init__(self, rect, sprite_path, shooter, owner=None, damage=0.1, visible=True, entity_id=None,
                 velocity=Vector2(0, 0), scale_res=None):
        super().__init__(rect=rect, sprite_path=sprite_path, velocity=velocity, scale_res=scale_res, owner=owner,
                         visible=visible, entity_id=entity_id)

        self.hit_sound = "assets/sounds/hit.mp3"
        self.shooter = shooter
        self.damage = damage

        self.entity_type = "bullet"

        self.update_funcs.append(self.apply_friction)
        self.update_funcs.append(self.despawn)
        self.update_funcs.extend((self.damage_agents,))
        #self.update_funcs.append(self.report)

    @staticmethod
    def create_from_dict(entity_dict):

        rect = Rect(entity_dict["rect"])
        sprite_path = entity_dict["sprite_path"]
        owner = entity_dict["owner"]
        visible = entity_dict["visible"]
        entity_id = entity_dict["entity_id"]
        velocity = Vector2(entity_dict["velocity"])
        scale_res = entity_dict["scale_res"]

        shooter = entity_dict["shooter"]
        damage = entity_dict["damage"]

        new_entity = Bullet(rect=rect, sprite_path=sprite_path, owner=owner, visible=visible, entity_id=entity_id,
                            velocity=velocity, scale_res=scale_res, shooter=shooter, damage=damage)

        return new_entity

    def dump_to_dict(self):

        data_dict = super().dump_to_dict()

        data_dict["shooter"] = self.shooter
        data_dict["damage"] = self.damage

        return data_dict

    def load_update(self, update_data):

        super().load_update(update_data)

        for key in update_data.keys():

            match key:
                case "shooter":
                    self.shooter = update_data["shooter"]
                case "damage":
                    self.damage = update_data["damage"]

    def apply_friction(self, state):

        #print(self.velocity)a
        self.velocity += -0.1 * self.velocity

        if abs(self.velocity.x) + abs(self.velocity.y) < 0.001:
            self.velocity = Vector2(0, 0)

    def damage_agents(self, state):

        collisions = self.detect_collisions(state.entities)

        #print(collisions)

        for entity in collisions:
            if type(entity) != Agent:
                continue
            elif entity.owner == self.owner:
                continue

            entity.health -= self.damage

            self.delete = True

            state.sounds.append(self.hit_sound)

            # update the server
            health_update = create_update("update", entity_id=entity.entity_id, data={"health": entity.health})
            despawn_update = create_update("delete", entity_id=self.entity_id)
            sound_update = create_update("sound", data={"path": self.hit_sound})

            state.updates.extend((health_update, despawn_update, sound_update))

    def despawn(self, state):

        if abs(self.velocity.x) + abs(self.velocity.y) < 1:
            self.delete = True

            update = create_update("delete", entity_id=self.entity_id)

            state.updates.append(update)


class Zombie(Entity):
    def __init__(self, rect, sprite_path, owner=None, visible=True, entity_id=None, velocity=Vector2(0, 0),
                 scale_res=None):

        super().__init__(rect=rect, sprite_path=sprite_path, velocity=velocity, scale_res=scale_res, owner=owner,
                         visible=visible, entity_id=entity_id)

class Agent(Entity):
    def __init__(self, rect, sprite_path, health=100, owner=None, visible=True, entity_id=None, acceleration=2, velocity=Vector2(0, 0), scale_res=None):

        # print(type(velocity))

        super().__init__(rect=rect, sprite_path=sprite_path, entity_id=entity_id, visible=visible, owner=owner,
                         velocity=velocity, scale_res=scale_res)

        self.acceleration = acceleration

        self.last_fire = 0

        self.update_funcs.extend((self.fire_bullet, self.accelerate, self.apply_friction, self.place_block))

        self.health = health

        self.entity_type = "agent"

        self.has_shot = False

    @staticmethod
    def create_from_dict(entity_dict):
        rect = Rect(entity_dict["rect"])
        sprite_path = entity_dict["sprite_path"]
        owner = entity_dict["owner"]
        visible = entity_dict["visible"]
        entity_id = entity_dict["entity_id"]
        velocity = Vector2(entity_dict["velocity"])
        scale_res = entity_dict["scale_res"]

        health = entity_dict["health"]
        acceleration = entity_dict["acceleration"]

        new_entity = Agent(rect=rect, sprite_path=sprite_path, owner=owner, visible=visible, entity_id=entity_id,
                           velocity=velocity, scale_res=scale_res, health=health, acceleration=acceleration)
    def dump_to_dict(self):
        data_dict = super().dump_to_dict()

        data_dict["acceleration"] = self.acceleration
        data_dict["health"] = self.health

        return data_dict

    def load_update(self, update_data):

        super().load_update(update_data)

        for key in update_data.keys():
            match key:
                case "acceleration":
                    self.acceleration = update_data["acceleration"]

                case "health":
                    self.health = update_data["health"]

    def place_block(self, state):

        keys = pygame.key.get_pressed()

        if pygame.mouse.get_pressed()[0] and keys[pygame.K_LSHIFT]:

            mouse_pos = pygame.mouse.get_pos()

            block_pos = (round_down(mouse_pos[0]), round_down(mouse_pos[1]))

            print(block_pos)

            new_block = Block(rect=Rect(block_pos[0], block_pos[1], 20, 20), sprite_path="assets/square.png", owner=self.owner, scale_res=(20,20))

            state.entities[new_block.entity_id] = new_block


    def fire_bullet(self, state):

        # if the user has already shot and wants to shoot, we dont let them
        if self.has_shot == True and pygame.mouse.get_pressed()[0] == True:
            return

        # if the user has not already shot and does not want to shoot
        elif self.has_shot == False and pygame.mouse.get_pressed()[0] == False:
            return

        # if the user does not want to shoot but shot last frame
        elif self.has_shot == True and pygame.mouse.get_pressed()[0] == False:
            self.has_shot = False

            return

        # we want to fire the bullet towards the mouse
        mouse_x, mouse_y = pygame.mouse.get_pos()

        # calculate the vector between the player and the player mouse
        delta_x = mouse_x - self.rect.x
        delta_y = mouse_y - self.rect.y

        # create a vector using delta values
        bullet_velocity = Vector2(delta_x, delta_y).normalize() * 50

        bullet = Bullet(rect=Rect(self.rect.center[0], self.rect.center[1], 1, 1),
                        shooter=self.entity_id,
                        velocity=self.velocity + bullet_velocity, sprite_path="assets/bullet.png",
                        scale_res=(15, 15), owner=self.owner)

        self.has_shot = True

        # # spawn 5 bullets per frame
        # for i in range(3):
        #     # now we need to give the bullet a velocity that makes it move towards the mouse but not instantly travel
        #     # if we set the bullet velocity equal to its path vector it would travel their instantly
        #     # we fix this by normalizing the vector to make the magnitude 1, but maintain its direction
        #     bullet_direction_vector = bullet_path_vector.normalize() * 25
        #
        #     # add some randomness to where the bullet spawns
        #     x_offset = random.gauss(3, 20)
        #
        #     y_offset = random.gauss(3, 20)
        #
        #     #print(self.velocity + bullet_direction_vector)
        #
        #     bullet = Bullet(rect=Rect(self.rect.center[0] + x_offset, self.rect.center[1] + y_offset, 1, 1),
        #                     shooter=self.entity_id,
        #                     velocity=self.velocity + bullet_direction_vector, sprite_path="assets/bullet.png",
        #                     scale_res=(15, 15), owner=self.owner)

        #print(bullet.velocity)

        state.entities[bullet.entity_id] = bullet

        # create update for the creation of the bullet
        update = create_update("create", entity_type="bullet", data=bullet.dump_to_dict())

        state.updates.append(update)

    def accelerate(self, state):
        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:
            self.velocity.y -= self.acceleration

        if keys[pygame.K_s]:
            self.velocity.y += self.acceleration

        if keys[pygame.K_a]:
            self.velocity.x -= self.acceleration

        if keys[pygame.K_d]:
            self.velocity.x += self.acceleration

        #(self.health)

    def get_angle_towards_mouse(self, state):

        mouse_x, mouse_y = pygame.mouse.get_pos()

        dx = mouse_x - self.rect.x
        dy = mouse_y - self.rect.y
        rads = math.atan2(-dy, dx)
        rads %= 2 * math.pi
        degs = math.degrees(rads)

        return degs

    def apply_friction(self, state):

        # print(self.velocity)

        self.velocity += -0.1 * self.velocity

        if abs(self.velocity.x) + abs(self.velocity.y) < 0.001:
            self.velocity = Vector2(0, 0)


class Game:
    def __init__(self, is_server=True):

        pygame.init()
        mixer.init()

        self.is_server = is_server

        if is_server:
            self.client_accepter = headered_socket.HeaderedSocket(socket.AF_INET)
            self.client_accepter.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.client_accepter.bind((socket.gethostname(), 5570))
            self.client_accepter.listen(5)
            self.client_accepter.setblocking(False)

        self.server = headered_socket.HeaderedSocket(socket.AF_INET)

        # our client ID
        self.uuid = str(uuid.uuid4())

        # our screen surface
        self.screen = pygame.display.set_mode([1280, 720])

        self.clock = pygame.time.Clock()

        # we use this map to translate network update types to actual entity types
        self.entity_map = {
            "agent": Agent,
            "bullet": Bullet,
            "entity": Entity
        }

        # we store this as a dict like entity_id:object
        # this makes it faster to lookup entities by their id
        self.state = State()

        # is our client connected to a server
        self.connected = False

    def accept_clients(self):

        clients = []

        while True:

            try:
                # try to accept the client socket
                client, address = self.client_accepter.accept()

                # send the new client a bunch of create updates with all the current entities we have
                initial_create_updates = []

                for entity_id, entity in self.state.entities.items():
                    # make a create update for each entity
                    update = create_update("create", entity_type=entity.entity_type, data=entity.dump_to_dict())

                    initial_create_updates.append(update)

                # convert the updates to json
                initial_create_updates_json = json.dumps(initial_create_updates)

                # only send the initial create updates if there are existing entities
                if initial_create_updates != []:
                    # send the updates to the client
                    client.send_headered(
                        bytes(initial_create_updates_json, "utf-8")
                    )

                    print(f"Sent initial update containing {len(initial_create_updates)} entities")

                # add the client to our list of client sockets
                clients.append(client)

            except BlockingIOError:
                pass
                # if there are no connections to accept then we just skip it

            for updating_client in clients:

                try:
                    # see if we have an update from the client
                    updates = updating_client.recv_headered()

                    #print(updates.decode("utf-8"))

                except BlockingIOError:
                    continue  # continue to the next client if we didnt receive an update

                # forward the updates to all the clients
                for client in clients:

                    # dont send the update to the client that sent it
                    if client is updating_client:
                        continue

                    client.send_headered(updates)

    def send_update(self, update):

        if update != []: # if they did not provide a list of updates, we put it in a list for them
            update = [update]

        update_json = json.dumps(update)

        update_bytes = bytes(update_json, "utf-8")

        self.server.send_headered(update_bytes)

    def dump_state(self):

        state_dict = {}

        for entity_id, entity in self.state.entities.items():
            state_dict[entity_id] = entity.dump_to_dict()

        return state_dict

    def connect(self, ip):

        # connect the server socket
        self.server.connect((ip, 5570))

        self.server.setblocking(False)  # only disable blocking when we are fully connected

        self.connected = True

    def receive_network_updates(self):

        # if we arent connected then we dont receive updates
        if not self.connected:
            return

        # get updates from the server
        try:
            server_updates_json = self.server.recv_headered().decode("utf-8")
        except BlockingIOError:
            return  # if there is no updates to read, then we skip

        server_updates = json.loads(server_updates_json)

        #print(server_updates)

        for update in server_updates:

            match update["update_type"]:

                case "create":

                    #print("Create!")

                    # get entity that is to be created
                    entity_class = self.entity_map[
                        update["entity_type"]
                    ]

                    new_entity = entity_class.create_from_dict(
                        update["data"]
                    )

                    #print(update["data"])
                    # add the new entity to the state
                    self.state.entities[new_entity.entity_id] = new_entity

                case "update":

                    #print("Update!")

                    # find the entity in the state and update it with the provided data
                    self.state.entities[update["entity_id"]].load_update(update["data"])

                case "delete":

                    #print("Delete!")

                    del self.state.entities[update["entity_id"]]

                case "sound":

                    print("new sounds")

                    self.state.sounds.append(
                        update["data"]["path"]
                    )

    def start(self):
        # creates all the initial entities we need to play

        local_agent = Agent(rect=Rect(100, 100, 50, 50), sprite_path="assets/square.png", acceleration=1,
                            owner=self.uuid, scale_res=(50, 50))

        self.state.entities[local_agent.entity_id] = local_agent

        update = create_update("create", entity_type="agent", data = local_agent.dump_to_dict())

        self.send_update(update)

    def update(self):

        self.screen.fill((0, 0, 0))

        # updates that will be sent this frame
        self.state.updates = []

        # sounds that will be played this frame
        self.state.sounds = []

        self.receive_network_updates()

        #print(self.state.sounds)

        for entity_id, entity in copy(self.state.entities).items():

            if entity.delete:
                del self.state.entities[entity_id]

                continue

            if entity.owner == self.uuid:  # only update the entity if we own it
                for function in entity.update_funcs:
                    function(self.state)

            if entity.visible:
                self.screen.blit(entity.sprite, entity.rect)

        for sound in self.state.sounds:
            sound_object = mixer.Sound(sound)

            sound_object.play()

            print("playing!")

        pygame.display.update()

        # if we have no updates then we dont send one
        if self.state.updates == []:
            return

        updates_json = json.dumps(self.state.updates)

        if self.connected:  # only send update if we are connected
            # send our update the server
            self.server.send_headered(
                bytes(updates_json, "utf-8")
            )

    @staticmethod
    def detect_changes(before_dict, after_dict):
        # detects the differences in the dicts and converts them to network updates

        # updates = [
        #     {
        #         "update_type": "update",
        #         "entity_id": "sample_entity_id",
        #         "data": {
        #             "position": [5,5]
        #         }
        #     }
        # ]

        updates = []

        for entity_id, entity in after_dict.items():

            # CREATE
            # if we find an entity that was not in the before dict, we make a CREATE update
            if entity_id not in list(before_dict.keys()):
                update = {
                    "update_type": "create",
                    "object_type": entity.object_type,
                    "data": entity.dump_to_dict()
                }

                updates.append(update)

                print(f"NEW ENTITY {entity_id}")

                continue

            # if this isnt a new entity, then we check to see if it has been updated
            else:
                update = {}

                for variable_name, value in entity.variable_map.items():

                    # print(
                    #     f"Comparing {variable_name}. OLD: {before_dict[entity_id].variable_map[variable_name]}, NEW: {value} ")

                    # print(variable_name)

                    # check if this value is the same as the one in the before dict
                    if value != before_dict[entity_id].variable_map[variable_name]:
                        print(f"Variable {variable_name} had updated value {value}!")

                        # look at the existing network update and add this variable if was different
                        update["update"].update(
                            {
                                variable_name: value
                            }
                        )

                # if there were no updates, then we dont send it
                if update == {}:
                    continue

                updates.append(update)


        # DELETE
        for entity_id, entity in before_dict.items():

            # if we find that an entity has been deleted then we create a delete update
            if entity_id not in list(after_dict.keys()):
                # make an update saying that we are deleting this entity
                update = {
                    "update_type": "delete",
                    "entity_id": entity_id
                }

                print("DELETED ENTITY")

        return updates

    def run(self):

        if self.is_server:
            new_client_thread = threading.Thread(target=self.accept_clients, daemon=True)
            new_client_thread.start()

        self.start()

        # self.connect(socket.gethostname())

        while True:

            events = pygame.event.get()

            for event in events:
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            #before_dict = frozendict(self.local_state)
            self.update()
            #after_dict = self.local_state

            #updates = self.detect_changes(before_dict, after_dict)

            # print(updates)

            self.clock.tick(60)

response = input("Host game? (y/N): ")

match response.lower():
    case "y":
        is_server = True

        ip = socket.gethostname()
        print("Hosting server!")

    case _:
        is_server = False

        ip = input("Enter IP: ")

        if ip == "":
            ip = socket.gethostname()

        print("Connecting to remote server")


game = Game(is_server=is_server)

game.connect(ip)

game.run()
