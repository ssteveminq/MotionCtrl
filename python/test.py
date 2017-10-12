import gym
env = gym.make('DartHopper-v1')
# env = gym.make('CartPole-v0')
observation = env.reset()
for i in range(100):
    observation, reward, done, envinfo = env.step(env.action_space.sample())
    env.render()
