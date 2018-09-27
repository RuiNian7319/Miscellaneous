import numpy as np

from copy import deepcopy


class ShootingRange:

    def __init__(self, nsim, nx, nu):

        """
        Nsim: Length of simulation
        Nx: Number of discrete states
        Nu: Number of discrete actions
        x0: Initial state
        states: Possible states
        actions: Possible actions

        x: State trajectory
        u: Input trajectory
        """

        self.Nsim = nsim
        self.Nx = nx
        self.Nu = nu
        self.states = np.linspace(0, 10, self.Nx)
        self.actions = np.linspace(-2, 2, self.Nu)
        self.x0 = np.random.randint(min(self.states), max(self.states))

        # Trajectories of the episodes

        self.x = np.zeros(self.Nsim)
        self.x[0] = self.x0
        self.u = np.zeros(self.Nsim)

    def __str__(self):
        return "A Shooting Range environment where we want the total to be 5"

    def __repr__(self):
        return "ShootingRange({}, {}, {})".format(self.Nsim, self.Nx, self.Nu)

    def step(self, actions, time):

        # Calculate the new state: x(t + 1) = x(t) + u(t)
        self.x[time] = self.x[time - 1] + actions
        self.u[time] = actions
        state = deepcopy(self.x[time])

        # Calculate the reward based on the current state
        reward = self.reward_calc(time)

        # Done returns true if it exceeds maximum episodes, if the states is greater than 10 or if target is hit.
        if time == self.Nsim:
            done = True
        elif self.x[time] > 10:
            done = True
        elif self.x[time] == 5:
            done = True
        else:
            done = False

        info = "placeholder"
        return state, reward, done, info

    """
    Reward function, 
    """

    def reward_calc(self, time):

        if self.x[time] == 5:
            reward = 1
        else:
            reward = 0

        return reward

    """
    Resets the state and input trajectories
    """

    def reset(self):

        self.x = np.zeros(self.Nsim)
        self.x[0] = np.random.randint(min(self.states), max(self.states))
        self.u = np.zeros(self.Nsim)

        return deepcopy(self.x[0])


if __name__ == "__main__":

    env = ShootingRange(nsim=15, nx=11, nu=5)
    episodes = 10

    for i in range(episodes):

        state = env.reset()

        for t in range(1, env.Nsim):
            State, Reward, Done, Info = env.step(np.random.choice(env.actions), t)

            if Done:
                break
