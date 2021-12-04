import numpy as np
from math import sin, cos, pi

from scipy.integrate import ode
from scipy.optimize import fsolve
from scipy.spatial import ConvexHull

from abc import ABC, abstractmethod

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse as EllipsePlot
from matplotlib.path import Path

class DynamicSystem(ABC):
    '''
    Abstract class for dynamic systems. This class has all the functionality for simulating dynamic systems using scipy integration methods.
    The abstract functionality that needs to be implemented by the child classes is the flow computation.
    '''
    def __init__(self, initial_state, initial_control):
        
        # Sets integration method from scipy
        self.mODE = ode(self.get_flow).set_integrator('dopri5')
        self.set_state(initial_state)
        self.set_control(initial_control)

        self.state_log = []
        for _ in range(0, self.n):
            self.state_log.append([])

        self.control_log = []
        for _ in range(0, self.m):
            self.control_log.append([])

    def set_state(self, state):
        '''
        Sets system state.
        '''
        self.n = len(state)
        self._state = np.array(state)
        self._dstate = np.zeros(self.n)
        self.mODE.set_initial_value(self._state)

    def set_control(self, control_input):
        '''
        Sets system control.
        '''
        self.m = len(control_input)
        self._control = np.array(control_input)

    def actuate(self, dt):
        '''
        Sends the control inputs.
        '''
        self.dynamics()
        self._state = self.mODE.integrate(self.mODE.t+dt)

        for state_dim in range(0, self.n):
            self.state_log[state_dim].append(self._state[state_dim])

        for ctrl_dim in range(0, self.m):
            self.control_log[ctrl_dim].append(self._control[ctrl_dim])

    def get_flow(self, t):
        '''
        Gets the current system flow, or state derivative.
        '''
        return self._dstate

    def get_state(self):
        '''
        Gets the current system state.
        '''
        return self._state

    def get_control(self):
        '''
        Gets the last control input.
        '''
        return self._control

    @abstractmethod
    def dynamics(self):
        pass


class AffineSystem(DynamicSystem):
    '''
    General class for an affine system dx = f(x) + g(x) u.
    '''
    def __init__(self, initial_state, initial_control):
        super().__init__(initial_state, initial_control)
        self._f = np.zeros(self.n)
        self._g = np.zeros([self.n, self.m])

    def get_f(self):
        '''
        Gets the value of f(x).
        '''
        return self._f

    def get_g(self):
        '''
        Gets the value of g(x).
        '''
        return self._g

    def dynamics(self):
        '''
        General affine system dynamics.
        '''
        self.f()
        self.g()
        self._dstate = self._f + self._g @ self._control

    @abstractmethod
    def f(self):
        pass

    @abstractmethod
    def g(self):
        pass


class Integrator(AffineSystem):
    '''
    Implements a simple n-order integrator.
    '''
    def __init__(self, initial_state, initial_control):
        if len(initial_state) != len(initial_control):
            raise Exception('Number of inputs is different than the number of states.')
        super().__init__(initial_state, initial_control)
        self.f()
        self.g()

    def f(self):
        self._f = np.zeros(self.n)

    def g(self):
        self._g = np.eye(self.n)


class LinearSystem(AffineSystem):
    '''
    Implements a linear system dx = A x + B u.
    '''
    def __init__(self, initial_state, initial_control, A, B):
        super().__init__(initial_state, initial_control)
        self._A = A
        self._B = B

    def f(self):
        self._f = self._A @ self._state

    def g(self):
        self._g = self._B


class Unicycle(AffineSystem):
    '''
    Implements the unicycle dynamics: dx = v cos(phi), dy = v sin(phi), dphi = omega.
    State and control are given by [x, y, z] and [v, omega], respectively.
    '''
    def __init__(self, initial_state, initial_control):
        if len(initial_state) != 3:
            raise Exception('State dimension is different from 3.')
        if len(initial_control) != 2:
            raise Exception('Control dimension is different from 3.')
        super().__init__(initial_state, initial_control)
        self.f()
        self.g()

    def f(self):
        self._f = np.zeros(self.n)

    def g(self):
        x = self._state[0]
        y = self._state[1]
        phi = self._state[2]
        self._g = np.array([[ np.cos(phi), 0.0 ],[ np.sin(phi), 0.0 ],[0.0, 1.0]])


class Ellipse():
    '''
    Implementation of elliptical obstacle class.
    '''
    def __init__(self, center = [0.0, 0.0], angle = 0.0, axes = [1.0, 1.0]):
        self.center = np.array(center)
        self.angle = angle
        self.axes = np.array(axes)
        self.point = np.zeros(2)

        s = np.sin(self.angle)
        c = np.cos(self.angle)
        self.R = np.array([[c,s],[-s,c]])

    def equations(self, vars):
        '''
        Equation for solving the 'nearest point to ellipse' problem. 
        '''
        l, gamma = vars

        a = self.axes[0]
        b = self.axes[1]

        term = self.R @ ( self.point - self.center )
        eq1 = (a + l*b)*cos(gamma) - term[0]
        eq2 = (b + l*a)*sin(gamma) - term[1]

        return [eq1, eq2]

    def compute_closest(self, point):
        '''
        Computes the closest point on the ellipse with respect to a given point.
        '''
        if self.isInside(point):
            return point, 0.0

        self.point = np.array(point)
        initial_l = np.random.rand()
        initial_gamma = pi*np.random.rand()
        l_sol, gamma_sol = fsolve(self.equations, (initial_l,initial_gamma))
        while l_sol < 0:
            initial_l = np.random.rand()
            initial_gamma = pi*np.random.rand()
            l_sol, gamma_sol = fsolve(self.equations, (initial_l, initial_gamma))

        a = self.axes[0]
        b = self.axes[1]

        v = np.array([ a*cos(gamma_sol), b*sin(gamma_sol) ])
        n = np.array([ b*cos(gamma_sol), a*sin(gamma_sol) ])
        closest_point = self.R.T @ v + self.center
        # distance = l_sol*np.linalg.norm(n)
        distance = np.linalg.norm(closest_point - point)

        return closest_point, distance

    def plot(self):
        ellipse = EllipsePlot(self.center, 2*self.axes[0], 2*self.axes[1], angle=np.degrees(self.angle), color='g', label='obstacle')
        plt.gca().add_patch(ellipse)

    def isInside(self, point):
        '''
        Checks if point is inside of ellipse.
        '''
        a = self.axes[0]
        b = self.axes[1]
        v = self.R @ ( np.array(point) - self.center )
        Lambda = np.diag([ 1/a**2, 1/b**2 ])
        expr = v.T @ Lambda @ v - 1

        return expr < 0


class Line():
    '''
    Implementation of simple line.
    '''
    def __init__(self, p1 = [0.0, 0.0], p2 = [1.0, 1.0]):
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.line_vector = self.p2 - self.p1
        self.angle = np.arctan2( self.line_vector[1], self.line_vector[0] )
        self.normal = np.array([ -sin(self.angle), cos(self.angle) ])

        self.point = np.zeros(2)

    def equations(self, vars):
        '''
        Equation for solving the 'nearest point to wall' problem. 
        '''
        l, gamma = vars

        eq1 = self.p1[0]*(1-gamma) + self.p2[0]*gamma + l*self.normal[0] - self.point[0]
        eq2 = self.p1[1]*(1-gamma) + self.p2[1]*gamma + l*self.normal[1] - self.point[1]

        return [eq1, eq2]

    def compute_closest(self, point):
        '''
        Computes the closest point on the line with respect to a given point.
        '''
        self.point = np.array(point)
        initial_l = np.random.rand()
        initial_gamma = np.random.rand()
        l_sol, gamma_sol = fsolve(self.equations, (initial_l,initial_gamma))
        distance = np.abs(l_sol)

        if gamma_sol > 1:
            gamma_sol = 1
            distance = np.linalg.norm( self.point - self.p2 )
        elif gamma_sol < 0:
            gamma_sol = 0
            distance = np.linalg.norm( self.point - self.p1 )

        closest_point = self.p1*(1-gamma_sol) + self.p2*gamma_sol

        return closest_point, distance

    def plot(self):
        '''
        Plot line.
        '''
        plt.plot(self.p1[0], self.p1[1], color='r', marker='*')
        plt.plot(self.p2[0], self.p2[1], color='r', marker='*')
        plt.plot([self.p1[0],self.p2[0]], [self.p1[1],self.p2[1]], color='b', linewidth=2.0)


class ConvexPolygon():
    '''
    Implementation of a convex polygon class. Vertices must be the vertices of a convex polygon.
    '''
    def __init__(self, vertices):
        self.vertices = np.array(vertices)
        self.num_vertices = len(self.vertices)
        self.lines = []

        self.create_lines()

        self.hull = ConvexHull( self.vertices )
        self.hull_path = Path( self.vertices[self.hull.vertices] )

    def create_lines(self):
        '''
        Create lines from vertice points.
        '''
        for k in range(self.num_vertices-1):
            self.lines.append( Line(p1 = self.vertices[k,:], p2 = self.vertices[k+1,:]) )
        self.lines.append( Line(p1 = self.vertices[-1,:], p2 = self.vertices[0,:]) )

    def isInside(self, point):
        '''
        Checks whether the point is inside the ConvexPolygon or not.
        '''
        return self.hull_path.contains_point(point)

    def compute_closest(self, point):
        '''
        Computes the closest point on the polygon to point.
        '''
        if self.isInside(point):
            return point, 0.0

        num_lines = len(self.lines)
        line_distances = np.zeros(num_lines)
        line_pts = np.zeros([2,num_lines])
        for k in range(num_lines):
            line_pts[:,k], line_distances[k] = self.lines[k].compute_closest(point)
        closest_line_index = np.argmin(line_distances)
        closest_line_pt = line_pts[:,closest_line_index]
        closest_line_dist = line_distances[closest_line_index]

        return closest_line_pt, closest_line_dist

    def plot(self):
        '''
        Plot polygon.
        '''
        for line in self.lines:
            line.plot()

class World():
    '''
    Implementation of simple class for a world with a polygonal arena and obstacles (elliptical and polygonal)
    '''
    def __init__(self, arena):
        self.arena = arena
        self.obstacles = []
        self.closest_object = None

    def add_obstacle(self, obs):
        '''
        Adds obstacles to the simulated world.
        '''
        self.obstacles.append(obs)

    def compute_closest(self, point):
        '''
        Computes the closest point on the obstacles or walls, with respect to point.
        '''
        # First, checks the arena walls
        closest_wall_dist = float('inf')
        num_walls = len(self.arena.lines)
        if num_walls > 0:
            wall_distances = np.zeros(num_walls)
            wall_pts = np.zeros([2,num_walls])
            for k in range(num_walls):
                wall_pts[:,k], wall_distances[k] = self.arena.lines[k].compute_closest(point)
            closest_wall_index = np.argmin(wall_distances)
            closest_wall_pt = wall_pts[:,closest_wall_index]
            closest_wall_dist = wall_distances[closest_wall_index]

        # Then, checks the obstacles
        closest_obs_dist = float('inf')
        num_obs = len(self.obstacles)
        if num_obs > 0:
            obs_distances = np.zeros(num_obs)
            obs_pts = np.zeros([2,num_obs])
            for k in range(num_obs):
                obs_pts[:,k], obs_distances[k] = self.obstacles[k].compute_closest(point)
            closest_obs_index = np.argmin(obs_distances)
            closest_obs_pt = obs_pts[:,closest_obs_index]
            closest_obs_dist = obs_distances[closest_obs_index]

        if closest_obs_dist < closest_wall_dist:
            closest_pt = closest_obs_pt
            closest_dist = closest_obs_dist
            self.closest_obstacle = self.obstacles[closest_obs_index]
        else:
            closest_pt = closest_wall_pt
            closest_dist = closest_wall_dist
            self.closest_obstacle = self.arena.lines[closest_wall_index]

        return closest_pt, closest_dist

    def plot(self):
        '''
        Plot the whole world.
        '''
        self.arena.plot()
        for obs in self.obstacles:
            obs.plot()


def check_collisions(points_vec, l_mtx, d_mtx, w_food_mtx, do, xi, time_score):

    c_mtx = d_mtx

    max_distance = 2
    if np.linalg.norm(xi[0]) > max_distance and l_mtx[1] == 0 and d_mtx[1] == 0: # Dist from 2 to 1
        print('Drone 2 lost...')
        l_mtx[1] = 1 
    elif np.linalg.norm(xi[0]) < max_distance and l_mtx[1] == 1 and d_mtx[1] == 0: # Dist from 2 to 1
        print('Drone 2 found!')
        l_mtx[1] = 0 
    
    if np.linalg.norm(xi[1]) > max_distance and l_mtx[2] == 0 and d_mtx[2] == 0: # Dist from 3 to 1
        print('Drone 3 lost...')
        l_mtx[2] = 1 
    elif np.linalg.norm(xi[1]) < max_distance and l_mtx[2] == 1 and d_mtx[2] == 0: # Dist from 3 to 1
        print('Drone 3 found!')
        l_mtx[2] = 0 

    if np.linalg.norm(xi[2]) > max_distance and l_mtx[3] == 0 and d_mtx[3] == 0: # Dist from 4 to 1
        print('Drone 4 lost...')
        l_mtx[3] = 1 
    elif np.linalg.norm(xi[2]) < max_distance and l_mtx[3] == 1 and d_mtx[3] == 0: # Dist from 4 to 1
        print('Drone 4 found!')
        l_mtx[3] = 0 

    if np.linalg.norm(xi[3]) > max_distance and l_mtx[4] == 0 and d_mtx[4] == 0: # Dist from 5 to 1
        print('Drone 5 lost...')
        l_mtx[4] = 1 
    elif np.linalg.norm(xi[3]) < max_distance and l_mtx[4] == 1 and d_mtx[4] == 0: # Dist from 5 to 1
        print('Drone 5 found!')
        l_mtx[4] = 0 
    
    
     
    
    for i in range(0,len(points_vec)-1):

        if d_mtx[i]==0: #para nao voltar a verificar -> pensar como implementar isto
            if points_vec[i][0] > 10 or points_vec[i][0] < -10 or points_vec[i][1] > 10 or points_vec[i][1] < -10: # Verify if the drone left the arena
                c_mtx[i]=1
                print('WARNING: Drone', i+1, 'left the arena!!')
                if w_food_mtx[i] == 1:
                    w_food_mtx[i] = 0
                    ind = np.where(w_food_mtx == 1)
                    ind_eli = ind[0][0]
                    w_food_mtx[ind_eli] = 0
                elif w_food_mtx[i] == 2:
                    w_food_mtx[i] = 0
                    ind2 = np.where(w_food_mtx == 1)
                    ind_eli2 = ind2[0][0]
                    w_food_mtx[ind_eli2] = 0
                continue

            elif do[i] < 0.01: # If distance to nearest object is smaller than 0.01, the drone is damaged - REVER PQ ELE PODE COLIDIR QUANDO NAO DETETA NENHUM OBSTACULO
                c_mtx[i]=1
                print('WARNING: Collision with obstacle! Drone:', i+1)
                if w_food_mtx[i] == 1:
                    w_food_mtx[i] = 0
                    ind = np.where(w_food_mtx == 1)
                    ind_eli = ind[0][0]
                    w_food_mtx[ind_eli] = 0
                elif w_food_mtx[i] == 2:
                    w_food_mtx[i] = 0
                    ind2 = np.where(w_food_mtx == 1)
                    ind_eli2 = ind2[0][0]
                    w_food_mtx[ind_eli2] = 0
                continue
            
            # Verify collisions between drones
            for j in range(i+1,4): 
                dist = np.sqrt( (points_vec[i][0] - points_vec[j][0])**2 + (points_vec[i][1] - points_vec[j][1])**2 ) 
                if dist < 0.1: 
                    c_mtx[i]=1
                    c_mtx[j]=1
                    print('WARNING: Colision between drone', i+1, 'and drone', j+1,'!!', ' -> dist =', dist)
                    if w_food_mtx[j] == 1:
                        w_food_mtx[j] = 0
                        ind = np.where(w_food_mtx == 1)
                        ind_eli = ind[0][0]
                        w_food_mtx[ind_eli] = 0
                        time_score[0] = 999
                        print('WARNING: Food 1 lost!')

                    if w_food_mtx[i] == 1:
                        w_food_mtx[i] = 0
                        ind = np.where(w_food_mtx == 1)
                        ind_eli = ind[0][0]
                        w_food_mtx[ind_eli] = 0
                        time_score[0] = 999
                        print('WARNING: Food 1 lost!')

                    if w_food_mtx[j] == 2:
                        w_food_mtx[j] = 0
                        ind2 = np.where(w_food_mtx == 2)
                        ind_eli2 = ind2[0][0]
                        w_food_mtx[ind_eli2] = 0
                        time_score[1] = 999
                        print('WARNING: Food 2 lost!')

                    if w_food_mtx[i] == 2:
                        w_food_mtx[i] = 0
                        ind2 = np.where(w_food_mtx == 2)
                        ind_eli2 = ind2[0][0]
                        w_food_mtx[ind_eli2] = 0
                        time_score[1] = 999
                        print('WARNING: Food 2 lost!')

                 
    d_mtx = c_mtx
    
    return l_mtx, d_mtx, w_food_mtx, time_score
    


def proximity(points_vec, food, num_foods):

    prox_mtx_1 = np.array([np.sqrt((points_vec[0][0] - food[1][0])**2 + (points_vec[0][1] - food[1][1])**2), 
                        np.sqrt((points_vec[1][0] - food[1][0])**2 + (points_vec[1][1] - food[1][1])**2),
                        np.sqrt((points_vec[2][0] - food[1][0])**2 + (points_vec[2][1] - food[1][1])**2),
                        np.sqrt((points_vec[3][0] - food[1][0])**2 + (points_vec[3][1] - food[1][1])**2),
                        np.sqrt((points_vec[4][0] - food[1][0])**2 + (points_vec[4][1] - food[1][1])**2)])
    if num_foods == 2:
        prox_mtx_2 = np.array([np.sqrt((points_vec[0][0] - food[2][0])**2 + (points_vec[0][1] - food[2][1])**2),
                            np.sqrt((points_vec[1][0] - food[2][0])**2 + (points_vec[1][1] - food[2][1])**2),
                            np.sqrt((points_vec[2][0] - food[2][0])**2 + (points_vec[2][1] - food[2][1])**2),
                            np.sqrt((points_vec[3][0] - food[2][0])**2 + (points_vec[3][1] - food[2][1])**2),
                            np.sqrt((points_vec[4][0] - food[2][0])**2 + (points_vec[4][1] - food[2][1])**2)])
    else:
        prox_mtx_2 = np.array([999,999,999,999,999])


    return prox_mtx_1, prox_mtx_2
    
    
    
def closest_drones(prox_mtx_1, prox_mtx_2, w_food_mtx, time_score, t, food_delivered, looking_for, num_foods):
    # Find the closest drones to the food
    closer = 99
    pre_candidate = 9
    candidate = 9
    #if prox_mtx_1[np.argmin(prox_mtx_1)] < 0.5 and w_food_mtx[np.argmin(prox_mtx_1)] == 0 and food_delivered[0] == 0: # If closer than 0.5 and food is not delivered

    for i in range(0, 4):
        if prox_mtx_1[i] < closer and w_food_mtx[i] == 0: # Finds the closest drone without food        
            closer = prox_mtx_1[i]
            pre_candidate = i
    
    closer = 99

    if pre_candidate != 9: # If a drone withou food was found and is the closest
        if prox_mtx_1[pre_candidate] < 0.5 and not 1 in w_food_mtx and food_delivered[0] == 0 and looking_for == 1: # If distance to the food is smaller than 0.5 and food 1 is not delivered and isn't with any other drone
            print('Drone',pre_candidate+1,'found the food 1')
            for i in [x for x in range(0,4) if x != pre_candidate and w_food_mtx[x] == 0]: # Verifies all drones except the closest one, and the ones with food
                if prox_mtx_1[i] < 0.5 and prox_mtx_1[i] < closer and w_food_mtx[i] == 0: # If closer than 0.5, closer than closer (to get the minimum) and without food
                    closer = prox_mtx_1[i]
                    candidate = i
            if candidate != 9:
                print('Drone',candidate+1,'found the food 1')
                w_food_mtx[pre_candidate] = 1
                w_food_mtx[candidate] = 1
                time_score[0] = t
            else:
                print('But no other drone found the food 1...')
    
    if num_foods == 2:  
        closer = 99
        pre_candidate = 9
        candidate = 9

        for i in range(0, 4):
            if prox_mtx_2[i] < closer and w_food_mtx[i] == 0: # Finds the closest drone without food   
                closer = prox_mtx_1[i]
                pre_candidate = i

        closer = 99
        
        if pre_candidate != 9: # If a drone withou food was found and is the closest
            if prox_mtx_2[(pre_candidate)] < 0.5 and not 2 in w_food_mtx and food_delivered[1] == 0 and looking_for == 2: # If distance to the food is smaller than 0.5 and food 1 is not delivered and isn't with any other drone
                print('Drone',pre_candidate+1,'found the food 2')
                for i in [x for x in range(0,4) if x != pre_candidate and w_food_mtx[x] == 0]: # Verifies all drones except the closest one, and the ones with food
                    if prox_mtx_2[i] < 0.5 and prox_mtx_2[i] < closer and w_food_mtx[i] == 0: # If closer than 0.5, closer than closer (to get the minimum) and without food
                        closer = prox_mtx_2[i]
                        candidate = i
                if candidate != 9:
                    print('Drone',candidate+1,'found the food 2')
                    w_food_mtx[pre_candidate] = 2
                    w_food_mtx[candidate] = 2
                    time_score[1] = t
                else:
                    print('But no other drone found the food 2...')

    return w_food_mtx, time_score



def food_base_check(w_food_mtx, points_vec, food, time_score, t, food_delivered):
    # Verify if food reached base or lost food
    if 1 in w_food_mtx:    
        result = np.where(w_food_mtx == 1)
        index = result[0][0]
        index_2 = result[0][1]

        if points_vec[index][0] < -5 and points_vec[index][1] < -5 and points_vec[index_2][0] < -5 and points_vec[index_2][1] < -5: # Arriving to base
            food[1][0] = points_vec[index][0]
            food[1][1] = points_vec[index][1]
            time_score[2] = t
            w_food_mtx[index] = 0
            w_food_mtx[index_2] = 0
            food_delivered[0] = 1
            print('Food 1 delivered!')
        elif np.sqrt( (points_vec[index][0] - points_vec[index_2][0])**2 + (points_vec[index][1] - points_vec[index_2][1])**2 ) > 1.5: # Loosing food
            w_food_mtx[index] = 0
            w_food_mtx[index_2] = 0
            time_score [0] = 999 
            print('WARNING: Food 1 lost!')

    if 2 in w_food_mtx:
        result2 = np.where(w_food_mtx == 2)
        index2 = result2[0][0]
        index2_2 = result2[0][1]

        if points_vec[index2][0] < -5 and points_vec[index2][1] < -5 and points_vec[index2_2][0] < -5 and points_vec[index2_2][1] < -5: # Arriving to base
            food[2][0] = points_vec[index2][0]
            food[2][1] = points_vec[index2][1]
            time_score[3] = t
            w_food_mtx[index2] = 0
            w_food_mtx[index2_2] = 0
            food_delivered[1] = 1
            print('Food 2 delivered!')
        elif np.sqrt( (points_vec[index2][0] - points_vec[index2_2][0])**2 + (points_vec[index2][1] - points_vec[index2_2][1])**2 ) > 1.5: # Loosing food
            w_food_mtx[index2] = 0
            w_food_mtx[index2_2] = 0
            time_score [1] = 999 
            print('WARNING: Food 2 lost!')
    
    return w_food_mtx, time_score, food_delivered