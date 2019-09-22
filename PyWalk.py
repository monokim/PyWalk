import pygame
import pymunk
import pymunk.pygame_util
import os
import math
import sys
import neat
import random

screen_width = 1700
screen_height = 960
space = pymunk.Space()
space.gravity = (0.0, -900.0)
generation = 0

class Robot:
    def __init__(self):
        self.tick = 0

        moment = 1
        friction = 1000
        self.shape = pymunk.Poly.create_box(None, (50, 100))
        body_moment = pymunk.moment_for_poly(moment, self.shape.get_vertices())
        self.body = pymunk.Body(moment, body_moment)
        #self.body = pymunk.Body(body_type=pymunk.Body.STATIC)

        self.body.position = (200, 350)
        self.shape.body = self.body
        self.shape.color = (150, 150, 150, 0)

        head_moment = pymunk.moment_for_circle(moment, 0, 30)
        self.head_body = pymunk.Body(moment, head_moment)
        self.head_body.position = (self.body.position.x, self.body.position.y+80)
        self.head_shape = pymunk.Circle(self.head_body, 30)
        self.head_joint = pymunk.PivotJoint(self.head_body, self.body, (-5, -30), (-5, 50))
        self.head_joint2 = pymunk.PivotJoint(self.head_body, self.body, (5, -30), (5, 50))


        arm_size = (100, 20)
        self.left_arm_upper_shape = pymunk.Poly.create_box(None, arm_size)
        left_arm_upper_moment = pymunk.moment_for_poly(moment, self.left_arm_upper_shape.get_vertices())
        self.left_arm_upper_body = pymunk.Body(moment, left_arm_upper_moment)
        self.left_arm_upper_body.position = (self.body.position.x-30, self.body.position.y)
        self.left_arm_upper_shape.body = self.left_arm_upper_body
        self.left_arm_upper_shape.friction = friction
        self.left_arm_upper_joint = pymunk.PivotJoint(self.left_arm_upper_body, self.body, (arm_size[0] / 2, 0), (-25, 30))
        self.la_motor = pymunk.SimpleMotor(self.body, self.left_arm_upper_body, 0)

        self.right_arm_upper_shape = pymunk.Poly.create_box(None, arm_size)
        right_arm_upper_moment = pymunk.moment_for_poly(moment, self.right_arm_upper_shape.get_vertices())
        self.right_arm_upper_body = pymunk.Body(moment, right_arm_upper_moment)
        self.right_arm_upper_body.position = (self.body.position.x+30, self.body.position.y)
        self.right_arm_upper_shape.body = self.right_arm_upper_body
        self.right_arm_upper_shape.friction = friction
        self.right_arm_upper_joint = pymunk.PivotJoint(self.right_arm_upper_body, self.body, (-arm_size[0] / 2, 0), (25, 30))
        self.ra_motor = pymunk.SimpleMotor(self.body, self.right_arm_upper_body, 0)

        thigh_size = (30, 60)
        self.lu_shape = pymunk.Poly.create_box(None, thigh_size)
        lu_moment = pymunk.moment_for_poly(moment, self.lu_shape.get_vertices())
        self.lu_body = pymunk.Body(moment, lu_moment)
        self.lu_body.position = (self.body.position.x-20, self.body.position.y-50)
        self.lu_shape.body = self.lu_body
        self.lu_joint = pymunk.PivotJoint(self.lu_body, self.body, (0, thigh_size[1] / 2), (-20, -50))
        self.lu_motor = pymunk.SimpleMotor(self.body, self.lu_body, 0)

        self.ru_shape = pymunk.Poly.create_box(None, thigh_size)
        ru_moment = pymunk.moment_for_poly(moment, self.ru_shape.get_vertices())
        self.ru_body = pymunk.Body(moment, ru_moment)
        self.ru_body.position = (self.body.position.x+20, self.body.position.y - 50)
        self.ru_shape.body = self.ru_body
        self.ru_joint = pymunk.PivotJoint(self.ru_body, self.body, (0, thigh_size[1] / 2), (20, -50))
        self.ru_motor = pymunk.SimpleMotor(self.body, self.ru_body, 0)

        leg_size = (20, 70)
        self.ld_shape = pymunk.Poly.create_box(None, leg_size)
        ld_moment = pymunk.moment_for_poly(moment, self.ld_shape.get_vertices())
        self.ld_body = pymunk.Body(moment, ld_moment)
        self.ld_body.position = (self.lu_body.position.x, self.lu_body.position.y - 100)
        self.ld_shape.body = self.ld_body
        self.ld_shape.friction = friction
        self.ld_joint = pymunk.PivotJoint(self.ld_body, self.lu_body, (0, leg_size[1] / 2), (0, -thigh_size[1] / 2))
        self.ld_motor = pymunk.SimpleMotor(self.lu_body, self.ld_body, 0)

        self.rd_shape = pymunk.Poly.create_box(None, leg_size)
        rd_moment = pymunk.moment_for_poly(moment, self.rd_shape.get_vertices())
        self.rd_body = pymunk.Body(moment, rd_moment)
        self.rd_body.position = (self.ru_body.position.x, self.ru_body.position.y - 100)
        self.rd_shape.body = self.rd_body
        self.rd_shape.friction = friction
        self.rd_joint = pymunk.PivotJoint(self.rd_body, self.ru_body, (0, leg_size[1] / 2), (0, -thigh_size[1] / 2))
        self.rd_motor = pymunk.SimpleMotor(self.ru_body, self.rd_body, 0)

        space.add(self.body, self.shape, self.head_body, self.head_shape, self.head_joint, self.head_joint2)
        space.add(self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor)
        space.add(self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor)
        space.add(self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor)
        space.add(self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor)
        space.add(self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor)
        space.add(self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor)


        shape_filter = pymunk.ShapeFilter(group=1)
        self.shape.filter = shape_filter
        self.head_shape.filter = shape_filter
        self.left_arm_upper_shape.filter = shape_filter
        self.right_arm_upper_shape.filter = shape_filter
        self.lu_shape.filter = shape_filter
        self.ru_shape.filter = shape_filter
        self.ld_shape.filter = shape_filter
        self.rd_shape.filter = shape_filter

        self.face = pygame.image.load('normal.png')
        self.face = pygame.transform.scale(self.face, (100, 100))

        self.is_done = False
        self.distance = 0
        self.check_point = 0

        self.lu_flag = False
        self.ld_flag = False
        self.ru_flag = False
        self.rd_flag = False
        self.la_flag = False
        self.ra_flag = False

    def get_shapes(self):
        body = self.body, self.shape
        head = self.head_body, self.head_shape, self.head_joint, self.head_joint2
        left_arm = self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor
        right_arm = self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor
        left_up_leg = self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor
        left_down_leg = self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor
        right_up_leg = self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor
        right_down_leg = self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor

        return body, head, left_arm, right_arm, left_up_leg, left_down_leg, right_up_leg, right_down_leg

    def get_data(self):
        lu = (360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle))
        ld = (360 - math.degrees(self.ld_body.angle)) - (360 - math.degrees(self.lu_body.angle))
        ru = (360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle))
        rd = (360 - math.degrees(self.rd_body.angle)) - (360 - math.degrees(self.ru_body.angle))
        arm_l = (360 - math.degrees(self.left_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle))
        arm_r = (360 - math.degrees(self.right_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle))
        return int(self.head_body.position.y), math.degrees(self.body.angle), lu, ld, ru, rd, arm_l, arm_r

    def draw_face(self, screen):
        rotated_face = rot_center(self.face, math.degrees(self.head_body.angle))
        screen.blit(rotated_face, (self.head_body.position[0] - 50, screen_height - self.head_body.position[1] - 50))

    def set_color(self, color, rest_color = (0, 0, 255)):
        self.shape.color = color
        self.head_shape.color = color
        self.left_arm_upper_shape.color = rest_color
        self.right_arm_upper_shape.color = rest_color
        self.lu_shape.color = rest_color
        self.ld_shape.color = rest_color
        self.ru_shape.color = rest_color
        self.rd_shape.color = rest_color

    def update(self):
        #lu
        self.lu_flag = False
        if (360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.lu_motor.rate > 0:
            self.lu_motor.rate = 0
            self.lu_flag = True
        elif (360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.lu_motor.rate < 0:
            self.lu_motor.rate = 0
            self.lu_flag = True

        #ld
        self.ld_flag = False
        if (360 - math.degrees(self.ld_body.angle)) - (360 - math.degrees(self.lu_body.angle)) >= 90 and self.ld_motor.rate > 0:
            self.ld_motor.rate = 0
            self.ld_flag = True
        elif (360 - math.degrees(self.ld_body.angle)) - (360 - math.degrees(self.lu_body.angle)) <= 0 and self.ld_motor.rate < 0:
            self.ld_motor.rate = 0
            self.ld_flag = True

        #ru
        self.ru_flag = False
        if (360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.ru_motor.rate > 0:
            self.ru_motor.rate = 0
            self.ru_flag = True
        elif (360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.ru_motor.rate < 0:
            self.ru_motor.rate = 0
            self.ru_flag = True

        #rd
        self.rd_flag = False
        if (360 - math.degrees(self.rd_body.angle)) - (360 - math.degrees(self.ru_body.angle)) >= 90 and self.rd_motor.rate > 0:
            self.rd_motor.rate = 0
            self.rd_flag = True
        elif (360 - math.degrees(self.rd_body.angle)) - (360 - math.degrees(self.ru_body.angle)) <= 0 and self.rd_motor.rate < 0:
            self.rd_motor.rate = 0
            self.rd_flag = True

        """
        #left arm
        self.la_flag = False
        if (360 - math.degrees(self.left_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.la_motor.rate > 0:
            self.la_motor.rate = 0
            self.la_flag = True
        elif (360 - math.degrees(self.left_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.la_motor.rate < 0:
            self.la_motor.rate = 0
            self.la_flag = True

        #right arm
        self.ra_flag = False
        if (360 - math.degrees(self.right_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) >= 90 and self.ra_motor.rate > 0:
            self.ra_motor.rate = 0
            self.ra_flag = True
        elif (360 - math.degrees(self.right_arm_upper_body.angle)) - (360 - math.degrees(self.body.angle)) <= -90 and self.ra_motor.rate < 0:
            self.ra_motor.rate = 0
            self.ra_flag = True
        """

        if self.head_body.position.y <= 250 or self.body.position.y <= 200 or self.tick > 120:
            self.is_done = True

    def add_space(self, space):
        space.add(self.body, self.shape, self.head_body, self.head_shape, self.head_joint)
        space.add(self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor)
        space.add(self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor)
        space.add(self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor)
        space.add(self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor)
        space.add(self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor)
        space.add(self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor)

    def set_position(self, x):
        self.body._set_position((self.body.position.x - x, self.body.position.y))
        self.head_body._set_position((self.head_body.position.x - x, self.head_body.position.y))
        self.left_arm_upper_body._set_position((self.left_arm_upper_body.position.x - x, self.left_arm_upper_body.position.y))
        self.right_arm_upper_body._set_position((self.right_arm_upper_body.position.x - x, self.right_arm_upper_body.position.y))

        self.lu_body._set_position((self.lu_body.position.x - x, self.lu_body.position.y))
        self.ru_body._set_position((self.ru_body.position.x - x, self.ru_body.position.y))
        self.ld_body._set_position((self.ld_body.position.x - x, self.ld_body.position.y))
        self.rd_body._set_position((self.rd_body.position.x - x, self.rd_body.position.y))

def add_land(space):
    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position = (0, 100)
    land = pymunk.Segment(body, (0, 0), (99999, 0), 10)
    land.friction = 100
    space.add(land)

def add_belt(space):

    self.right_arm_upper_shape = pymunk.Poly.create_box(None, arm_size)
    right_arm_upper_moment = pymunk.moment_for_poly(moment, self.right_arm_upper_shape.get_vertices())
    self.right_arm_upper_body = pymunk.Body(moment, right_arm_upper_moment)
    self.right_arm_upper_body.position = (self.body.position.x+30, self.body.position.y)
    self.right_arm_upper_shape.body = self.right_arm_upper_body
    self.right_arm_upper_shape.friction = friction
    self.right_arm_upper_joint = pymunk.PivotJoint(self.right_arm_upper_body, self.body, (-arm_size[0] / 2, 0), (25, 30))
    self.ra_motor = pymunk.SimpleMotor(self.body, self.right_arm_upper_body, 0)

    body = pymunk.Body(body_type=pymunk.Body.STATIC)
    body.position = (0, 100)
    land = pymunk.Segment(body, (0, 0), (99999, 0), 10)
    land.friction = 100
    space.add(land)

def rot_center(image, angle):
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image

def check_distance(robot, human):
    distance = math.sqrt(math.pow(robot[0] - human[0], 2) + math.pow(robot[1] - human[1], 2))
    return distance

def run_test():
    pygame.init()
    screen = pygame.display.set_mode((screen_width, screen_height))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    font = pygame.font.SysFont("Arial", 30)

    robot = Robot()
    add_land(space)

    #main game
    speed = 1
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    robot.rd_motor.rate = speed
                elif event.key == pygame.K_DOWN:
                    robot.rd_motor.rate = -speed
                elif event.key == pygame.K_RIGHT:
                    robot.ru_motor.rate = speed
                elif event.key == pygame.K_LEFT:
                    robot.ru_motor.rate = -speed

                if event.key == pygame.K_w:
                    robot.ld_motor.rate = speed
                elif event.key == pygame.K_s:
                    robot.ld_motor.rate = -speed
                elif event.key == pygame.K_a:
                    robot.lu_motor.rate = speed
                elif event.key == pygame.K_d:
                    robot.lu_motor.rate = -speed

        robot.update()
        space.step(1/50.0)
        screen.fill((255, 255, 255))
        space.debug_draw(draw_options)

        robot.draw_face(screen)
        pygame.display.flip()
        clock.tick(60)


def robot_walk(genomes, config):
    pygame.init()
    screen = pygame.display.set_mode((screen_width, screen_height))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    generation_font = pygame.font.SysFont("Arial", 70)
    font = pygame.font.SysFont("Arial", 30)

    ruler = 0

    nets = []
    ge = []
    robots = []

    for id, g in genomes:
        net = neat.nn.FeedForwardNetwork.create(g, config)
        nets.append(net)
        robots.append(Robot())
        g.fitness = 0
        ge.append(g)

    add_land(space)

    #main game
    global generation
    generation += 1
    if generation > 1000:
        draw_flag = True
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)


        if len(robots) == 0:
            break

        for i, robot in enumerate(robots):
            #ge[i].fitness += 0.1
            output = nets[i].activate(robot.get_data())
            speed = 3
            for i, out in enumerate(output):
                if out > 0.9:
                    if i == 0 and robot.lu_flag == False:
                        robot.lu_motor.rate = speed
                    elif i == 1 and robot.lu_flag == False:
                        robot.lu_motor.rate = -speed
                    elif i == 2 and robot.ru_flag == False:
                        robot.ru_motor.rate = speed
                    elif i == 3 and robot.ru_flag == False:
                        robot.ru_motor.rate = -speed
                    elif i == 4 and robot.ld_flag == False:
                        robot.ld_motor.rate = speed
                    elif i == 5 and robot.ld_flag == False:
                        robot.ld_motor.rate = -speed
                    elif i == 6 and robot.rd_flag == False:
                        robot.rd_motor.rate = speed
                    elif i == 7 and robot.rd_flag == False:
                        robot.rd_motor.rate = -speed
                    elif i == 8 and robot.la_flag == False:
                        robot.la_motor.rate = speed
                    elif i == 9 and robot.la_flag == False:
                        robot.la_motor.rate = -speed
                    elif i == 10 and robot.ra_flag == False:
                        robot.ra_motor.rate = speed
                    elif i == 11 and robot.ra_flag == False:
                        robot.ra_motor.rate = -speed

        # check
        max = 0
        at = 0
        for i, robot in enumerate(robots):
            robot.update()
            robot.set_color((240, 240, 240), (240, 240, 240))
            distance = robot.body.position.x - 200
            #distance = int(check_distance(robot.body.position, (200, 350)))
            threshold = 5
            if distance - robot.check_point < threshold:
                robot.tick += 1
            else:
                ge[i].fitness += 1
                robot.tick = 0
                robot.check_point = distance

            robot.distance = distance
            if distance > max:
                max = distance
                at = i

        robots[at].set_color((255, 0, 0))
        robot_position = robots[at].body.position

        if robots[at].body.position.x > screen_width / 2:
            d = int(robots[at].body.position.x - screen_width / 2)
            for robot in robots:
                robot.set_position(d)
            ruler -= 5

        space.step(1/50.0)
        screen.fill((255, 255, 255))
        space.debug_draw(draw_options)
        robots[at].draw_face(screen)

        text = generation_font.render("Generation : " + str(generation), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (screen_width/2, screen_height/2 - 100)
        screen.blit(text, text_rect)


        text = font.render("distance : " + str(int(robots[at].distance)), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (robots[at].body.position.x, screen_height/2 + 50)
        screen.blit(text, text_rect)

        for i in range(ruler, 1960, 100):
            pygame.draw.line(screen, (0, 0, 0), (i, screen_height - 100), (i, screen_height - 90))

        if ruler < 0:
            ruler += 100

        for i, robot in enumerate(robots):

            robot.lu_motor.rate = 0
            robot.ru_motor.rate = 0
            robot.ld_motor.rate = 0
            robot.rd_motor.rate =  0
            robot.la_motor.rate =  0
            robot.ra_motor.rate = 0

            if robot.is_done:
                ge[i].fitness -= 1
                space.remove(robot.get_shapes())
                robots.remove(robot)
                ge.pop(i)


        pygame.display.flip()
        clock.tick(60)

def run(config_path):
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)

    p = neat.Population(config)
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)

    generation = 0
    winner = p.run(robot_walk, 1000)


#run_test()

if __name__ == "__main__":
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config-feedforward.txt")
    run(config_path)
