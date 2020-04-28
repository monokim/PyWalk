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
        moment = 5
        friction = 0.5
        self.shape = pymunk.Poly.create_box(None, (50, 100))
        body_moment = pymunk.moment_for_poly(moment, self.shape.get_vertices())
        self.body = pymunk.Body(moment, body_moment)
        #self.body = pymunk.Body(body_type=pymunk.Body.STATIC)

        self.body.position = (screen_width/2, 500)
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
        self.left_arm_upper_joint = pymunk.PivotJoint(self.left_arm_upper_body, self.body, (arm_size[0] / 2, 0), (-25, 30))
        self.la_motor = pymunk.SimpleMotor(self.body, self.left_arm_upper_body, 0)

        self.right_arm_upper_shape = pymunk.Poly.create_box(None, arm_size)
        right_arm_upper_moment = pymunk.moment_for_poly(moment, self.right_arm_upper_shape.get_vertices())
        self.right_arm_upper_body = pymunk.Body(moment, right_arm_upper_moment)
        self.right_arm_upper_body.position = (self.body.position.x+30, self.body.position.y)
        self.right_arm_upper_shape.body = self.right_arm_upper_body
        self.right_arm_upper_joint = pymunk.PivotJoint(self.right_arm_upper_body, self.body, (-arm_size[0] / 2, 0), (25, 30))
        self.ra_motor = pymunk.SimpleMotor(self.body, self.right_arm_upper_body, 0)

        thigh_size = (30, 60)
        self.lu_shape = pymunk.Poly.create_box(None, thigh_size)
        lu_moment = pymunk.moment_for_poly(moment, self.lu_shape.get_vertices())
        self.lu_body = pymunk.Body(moment, lu_moment)
        self.lu_body.position = (self.body.position.x-20, self.body.position.y-50)
        self.lu_shape.body = self.lu_body
        self.lu_shape.friction = friction
        self.lu_joint = pymunk.PivotJoint(self.lu_body, self.body, (0, thigh_size[1] / 2), (-20, -50))
        self.lu_motor = pymunk.SimpleMotor(self.body, self.lu_body, 0)

        self.ru_shape = pymunk.Poly.create_box(None, thigh_size)
        ru_moment = pymunk.moment_for_poly(moment, self.ru_shape.get_vertices())
        self.ru_body = pymunk.Body(moment, ru_moment)
        self.ru_body.position = (self.body.position.x+20, self.body.position.y - 50)
        self.ru_shape.body = self.ru_body
        self.ru_shape.friction = friction
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


        foot_size = (45, 20)
        self.lf_shape = pymunk.Poly.create_box(None, foot_size)
        rd_moment = pymunk.moment_for_poly(moment, self.lf_shape.get_vertices())
        self.lf_body = pymunk.Body(moment, rd_moment)
        self.lf_body.position = (self.ld_body.position.x + foot_size[0]/2, self.ld_body.position.y + (foot_size[1]/2 + leg_size[1]/2))
        self.lf_shape.body = self.lf_body
        self.lf_shape.friction = friction
        self.lf_shape.mass = 30
        self.lf_joint = pymunk.PivotJoint(self.ld_body, self.lf_body, (-5, -leg_size[1] / 2), (-foot_size[0]/2 + 10, foot_size[1]/2))
        self.lf_motor = pymunk.SimpleMotor(self.ld_body, self.lf_body, 0)

        self.rf_shape = pymunk.Poly.create_box(None, foot_size)
        rd_moment = pymunk.moment_for_poly(moment, self.rf_shape.get_vertices())
        self.rf_body = pymunk.Body(moment, rd_moment)
        self.rf_body.position = (self.rd_body.position.x + foot_size[0]/2, self.rd_body.position.y + (foot_size[1]/2 + leg_size[1]/2))
        self.rf_shape.body = self.rf_body
        self.rf_shape.friction = friction
        self.rf_shape.mass = 30
        self.rf_joint = pymunk.PivotJoint(self.rd_body, self.rf_body, (-5, -leg_size[1] / 2), (-foot_size[0]/2 + 10, foot_size[1]/2))
        self.rf_motor = pymunk.SimpleMotor(self.rd_body, self.rf_body, 0)

        space.add(self.body, self.shape, self.head_body, self.head_shape, self.head_joint, self.head_joint2)
        #space.add(self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor)
        #space.add(self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor)
        space.add(self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor)
        space.add(self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor)
        space.add(self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor)
        space.add(self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor)
        space.add(self.lf_body, self.lf_shape, self.lf_joint, self.lf_motor)
        space.add(self.rf_body, self.rf_shape, self.rf_joint, self.rf_motor)


        shape_filter = pymunk.ShapeFilter(group=1)
        self.shape.filter = shape_filter
        self.head_shape.filter = shape_filter
        self.left_arm_upper_shape.filter = shape_filter
        self.right_arm_upper_shape.filter = shape_filter
        self.lu_shape.filter = shape_filter
        self.ru_shape.filter = shape_filter
        self.ld_shape.filter = shape_filter
        self.rd_shape.filter = shape_filter
        self.lf_shape.filter = shape_filter
        self.rf_shape.filter = shape_filter

        self.face = pygame.image.load('normal.png')
        self.face = pygame.transform.scale(self.face, (100, 100))

        self.no_face = pygame.image.load('no.png')
        self.no_face = pygame.transform.scale(self.no_face, (100, 100))

        self.is_done = False
        self.distance = 0

        self.la_flag = False
        self.ra_flag = False
        self.lu_flag = False
        self.ld_flag = False
        self.ru_flag = False
        self.rd_flag = False
        self.lf_flag = False
        self.rf_flag = False

    def get_shapes(self):
        body = self.body, self.shape
        head = self.head_body, self.head_shape, self.head_joint, self.head_joint2
        #left_arm = self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor
        #right_arm = self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor

        left_up_leg = self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor
        left_down_leg = self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor
        left_foot = self.lf_body, self.lf_shape, self.lf_joint, self.lf_motor
        right_up_leg = self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor
        right_down_leg = self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor
        right_foot = self.rf_body, self.rf_shape, self.rf_joint, self.rf_motor

        return body, head, left_up_leg, left_down_leg, left_foot, right_up_leg, right_down_leg, right_foot
        #return body, head, left_arm, right_arm, left_up_leg, left_down_leg, left_foot, right_up_leg, right_down_leg, right_foot

    def get_data(self):
        lu = ((360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        ld = ((360 - math.degrees(self.ld_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        lf = ((360 - math.degrees(self.lf_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        ru = ((360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        rd = ((360 - math.degrees(self.rd_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        rf = ((360 - math.degrees(self.rf_body.angle)) - (360 - math.degrees(self.body.angle))) / 360.0
        return self.body.angle, lu, ld, lf, ru, rd, rf

    def draw_face(self, screen):
        if self.body.position.x > 550:
            rotated_face = rot_center(self.face, math.degrees(self.head_body.angle))
        else:
            rotated_face = rot_center(self.no_face, math.degrees(self.head_body.angle))
        screen.blit(rotated_face, (self.head_body.position[0] - 50, screen_height - self.head_body.position[1] - 50))

    def set_color(self, color, rest_color = (0, 0, 255), shoe_color = (50, 50, 50)):
        self.shape.color = color
        self.head_shape.color = color
        #self.left_arm_upper_shape.color = rest_color
        #self.right_arm_upper_shape.color = rest_color
        self.lu_shape.color = rest_color
        self.ld_shape.color = rest_color
        self.lf_shape.color = shoe_color
        self.ru_shape.color = rest_color
        self.rd_shape.color = rest_color
        self.rf_shape.color = shoe_color

    def update(self):
        #lu
        self.lu_flag = False
        if (360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle)) >= 70 and self.lu_motor.rate > 0:
            self.lu_motor.rate = 0
            self.lu_flag = True
        elif (360 - math.degrees(self.lu_body.angle)) - (360 - math.degrees(self.body.angle)) <= -70 and self.lu_motor.rate < 0:
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

        #lf
        self.lf_flag = False
        if (360 - math.degrees(self.lf_body.angle)) - (360 - math.degrees(self.ld_body.angle)) >= 30 and self.lf_motor.rate > 0:
            self.lf_motor.rate = 0
            self.lf_flag = True
        elif (360 - math.degrees(self.lf_body.angle)) - (360 - math.degrees(self.ld_body.angle)) <= -45 and self.lf_motor.rate < 0:
            self.lf_motor.rate = 0
            self.lf_flag = True

        #ru
        self.ru_flag = False
        if (360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle)) >= 70 and self.ru_motor.rate > 0:
            self.ru_motor.rate = 0
            self.ru_flag = True
        elif (360 - math.degrees(self.ru_body.angle)) - (360 - math.degrees(self.body.angle)) <= -70 and self.ru_motor.rate < 0:
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

        #rf
        self.rf_flag = False
        if (360 - math.degrees(self.rf_body.angle)) - (360 - math.degrees(self.rd_body.angle)) >= 30 and self.rf_motor.rate > 0:
            self.rf_motor.rate = 0
            self.rf_flag = True
        elif (360 - math.degrees(self.rf_body.angle)) - (360 - math.degrees(self.rd_body.angle)) <= -45 and self.rf_motor.rate < 0:
            self.rf_motor.rate = 0
            self.rf_flag = True

        #self.la_flag = False
        #self.ra_flag = False

        if self.body.position.y <= 100 or (self.head_body.position.x > 450 and self.head_body.position.y <= 350):
            self.is_done = True
            space.remove(self.get_shapes())

    def add_space(self, space):
        space.add(self.body, self.shape, self.head_body, self.head_shape, self.head_joint)
        #space.add(self.left_arm_upper_body, self.left_arm_upper_shape, self.left_arm_upper_joint, self.la_motor)
        #space.add(self.right_arm_upper_body, self.right_arm_upper_shape, self.right_arm_upper_joint, self.ra_motor)
        space.add(self.lu_body, self.lu_shape, self.lu_joint, self.lu_motor)
        space.add(self.ru_body, self.ru_shape, self.ru_joint, self.ru_motor)
        space.add(self.ld_body, self.ld_shape, self.ld_joint, self.ld_motor)
        space.add(self.rd_body, self.rd_shape, self.rd_joint, self.rd_motor)
        space.add(self.lf_body, self.lf_shape, self.lf_joint, self.lf_motor)
        space.add(self.rf_body, self.rf_shape, self.rf_joint, self.rf_motor)

def add_land(space):
    land_size = (screen_width - 300, 20)
    shape = pymunk.Poly.create_box(None, land_size)
    shape.friction = 0.5
    shape.elasticity = 1.0
    moment = pymunk.moment_for_poly(1, shape.get_vertices())
    body = pymunk.Body(9999, moment, body_type=pymunk.Body.KINEMATIC)
    body.position = (screen_width/2 + 300, 300)
    shape.body = body
    space.add(body, shape)
    #shape.surface_velocity = (-100,0)

    return shape

def rot_center(image, angle):
    orig_rect = image.get_rect()
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = orig_rect.copy()
    rot_rect.center = rot_image.get_rect().center
    rot_image = rot_image.subsurface(rot_rect).copy()
    return rot_image

def robot_walk(genomes, config):
    pygame.init()
    screen = pygame.display.set_mode((screen_width, screen_height))
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    generation_font = pygame.font.SysFont("Arial", 70)
    font = pygame.font.SysFont("Arial", 30)


    foundry = pygame.image.load('foundry.png')
    foundry = pygame.transform.scale(foundry, (600, 350))

    ruler = 0
    nets = []
    robots = []

    for id, g in genomes:
        net = neat.nn.FeedForwardNetwork.create(g, config)
        nets.append(net)
        robots.append(Robot())
        g.fitness = 0

    land = add_land(space)
    land.surface_velocity = (-100, 0)

    #main game
    global generation
    generation += 1
    tick = 0
    speed_up = 1
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit(0)

        for i, robot in enumerate(robots):
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
                    elif i == 8 and robot.lf_flag == False:
                        robot.lf_motor.rate = speed
                    elif i == 9 and robot.lf_flag == False:
                        robot.lf_motor.rate = -speed
                    elif i == 10 and robot.rf_flag == False:
                        robot.rf_motor.rate = speed
                    elif i == 11 and robot.rf_flag == False:
                        robot.rf_motor.rate = -speed

        # check
        max = 0
        at = 0
        remain_robot = 0
        for i, robot in enumerate(robots):
            if not robot.is_done:
                remain_robot += 1
                robot.update()
                robot.set_color((240, 240, 240), (240, 240, 240), (240, 240, 240))
                genomes[i][1].fitness += 0.1
                distance = robot.body.position.x
                if distance > robot.distance:
                    genomes[i][1].fitness += 0.1
                robot.distance = distance
                if robot.body.position.x > max:
                    max = robot.body.position.x
                    at = i


        if remain_robot == 0:
            break

        robots[at].set_color((255, 0, 0))
        robot_position = robots[at].body.position
        ruler -= (speed_up+1)

        tick += 1
        if tick == 600:
            land.surface_velocity = (-100 * speed_up, 0)
            speed_up += 1
            tick = 0

        #space.step(1/50.0)
        space.step(1/50.0)
        screen.fill((255, 255, 255))
        screen.blit(foundry, (0, screen_height - 300))
        space.debug_draw(draw_options)
        robots[at].draw_face(screen)

        text = generation_font.render("Generation : " + str(generation), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (screen_width/2, 100)
        screen.blit(text, text_rect)

        text = font.render("Speed up (x" + (str(speed_up+1)) +") after : " + str(int((600 - tick) / 60)) + " sec", True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (screen_width/2, 200)
        screen.blit(text, text_rect)


        text = font.render("EXIT->", True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (screen_width-50, 500)
        screen.blit(text, text_rect)

        for i in range(ruler, 1700, 100):
            pygame.draw.line(screen, (0, 0, 0), (i, screen_height - 300), (i, screen_height - 290))

        if ruler < 450:
            ruler = 550

        for i, robot in enumerate(robots):
            if not robot.is_done:
                robot.lu_motor.rate = 0
                robot.ru_motor.rate = 0
                robot.ld_motor.rate = 0
                robot.rd_motor.rate =  0
                robot.lf_motor.rate = 0
                robot.rf_motor.rate =  0


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

if __name__ == "__main__":
    __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config-feedforward.txt")
    run(config_path)
