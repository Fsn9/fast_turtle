import math
import numpy as np
from scipy.integrate import ode
from abc import ABC, abstractmethod


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