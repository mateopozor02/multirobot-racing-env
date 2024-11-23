import re
import argparse
import matplotlib.pyplot as plt
import numpy as np


def running_mean(x, n):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[n:] - cumsum[:-n]) / float(n)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('log_files', type=str, nargs='+')
    parser.add_argument('--plot_sr', default=False, action='store_true')
    parser.add_argument('--plot_cr', default=False, action='store_true')
    parser.add_argument('--plot_time', default=False, action='store_true')
    parser.add_argument('--plot_reward', default=True, action='store_true')
    parser.add_argument('--plot_train', default=True, action='store_true')
    parser.add_argument('--plot_val', default=False, action='store_true')
    parser.add_argument('--window_size', type=int, default=300)
    args = parser.parse_args()

    # define the names of the models you want to plot and the longest episodes you want to show
    models = ['SARL*', 'model 2', 'model 3', 'model 4']
    models3 = ['model 3']
    max_episodes = 20000

    ax1 = ax2 = ax3 = ax4 = None
    ax1_legends = []
    ax2_legends = []
    ax3_legends = []
    ax4_legends = []

    for i, log_file in enumerate(args.log_files):
        with open(log_file, 'r') as file:
            log = file.read()

        val_pattern = r"VAL\s+in episode\s+(?P<episode>\d+)\s+has success rate:\s+(?P<sr>[01]\.\d+),\s+" \
              r"collision rate:\s+(?P<cr>[01]\.\d+),\s+timeout rate:\s+(?P<tr>[01]\.\d+),\s+" \
              r"collisionwall rate:\s+(?P<cwr>[01]\.\d+),\s+nav time:\s+(?P<time>\d+\.\d+),\s+" \
              r"average speed:\s+(?P<speed>\d+\.\d+),\s+path length:\s+(?P<pl>\d+\.\d+),\s+" \
              r"total reward:\s+(?P<reward>[-+]?\d+\.\d+)"

        val_episode = []
        val_sr = []
        val_cr = []
        val_tr = []
        val_cwr = []
        val_time = []
        val_speed = []
        val_pl = []
        val_reward = []
        for r in re.findall(val_pattern, log):
            #print(r)
            val_episode.append(int(r[0]))
            val_sr.append(float(r[1]))
            val_cr.append(float(r[2]))
            val_tr.append(float(r[3]))
            val_cwr.append(float(r[4]))
            val_time.append(float(r[5]))
            val_speed.append(float(r[6]))
            val_pl.append(float(r[7]))
            val_reward.append(float(r[8]))

        #print(val_reward)

        train_pattern = r"TRAIN in episode (?P<episode>\d+) has success rate: (?P<sr>[01]\.\d+),\s+" \
                r"collision rate: (?P<cr>[01]\.\d+),\s+timeout rate:\s+(?P<tr>[01]\.\d+),\s+" \
                r"collisionwall rate:\s+(?P<cwr>[01]\.\d+),\s+nav time:\s+(?P<time>\d+\.\d+),\s+" \
                r"average speed:\s+(?P<speed>\d+\.\d+),\s+path length:\s+(?P<pl>\d+\.\d+),\s+" \
                r"total reward:\s+(?P<reward>[-+]?\d+\.\d+)"
        
        train_episode = []
        train_sr = []
        train_cr = []
        train_tr = []
        train_cwr = []
        train_time = []
        train_speed = []
        train_pl = []
        train_reward = []
        for r in re.findall(train_pattern, log):
            train_episode.append(int(r[0]))
            train_sr.append(float(r[1]))
            train_cr.append(float(r[2]))
            train_tr.append(float(r[3]))
            train_cwr.append(float(r[4]))
            train_time.append(float(r[5]))
            train_speed.append(float(r[6]))
            train_pl.append(float(r[7]))
            train_reward.append(float(r[8]))
        train_episode = train_episode[:max_episodes]
        train_sr = train_sr[:max_episodes]
        train_cr = train_cr[:max_episodes]
        train_tr = train_tr[:max_episodes]
        train_cwr = train_cwr[:max_episodes]
        train_time = train_time[:max_episodes]
        train_speed = train_speed[:max_episodes]
        train_pl = train_pl[:max_episodes]
        train_reward = train_reward[:max_episodes]

        # smooth training plot
        train_sr_smooth = running_mean(train_sr, args.window_size)
        train_cr_smooth = running_mean(train_cr, args.window_size)
        train_time_smooth = running_mean(train_time, args.window_size)
        train_reward_smooth = running_mean(train_reward, args.window_size)

        # plot sr
        if args.plot_sr:
            if ax1 is None:
                _, ax1 = plt.subplots()
            if args.plot_train:
                ax1.plot(range(len(train_sr_smooth)), train_sr_smooth)
                ax1_legends.append("Training")
            if args.plot_val:
                #print("EXECUTING")
                ax1.plot(val_episode, val_sr)
                ax1_legends.append("Validation")

            ax1.legend(ax1_legends)
            ax1.set_xlabel('Number of episodes')
            ax1.set_ylabel('Success Rate')
            ax1.set_title("SARL*")
            plt.grid(True)
            plt.xlim([0, 5000])
            plt.ylim([0.2, 1.0]) 

        # plot time
        if args.plot_time:
            if ax2 is None:
                _, ax2 = plt.subplots()
            if args.plot_train:
                ax2.plot(range(len(train_time_smooth)), train_time_smooth)
                ax2_legends.append(models[i])
            if args.plot_val:
                ax2.plot(val_episode, val_time)
                ax2_legends.append(models[i])

            ax2.legend(ax2_legends)
            ax2.set_xlabel('Number of episodes')
            ax2.set_ylabel('Time to reach the goal(s)')
            ax2.set_title("")
            plt.grid(True)
            plt.ylim([5, 60]) #5,45

        # plot cr
        if args.plot_cr:
            if ax3 is None:
                _, ax3 = plt.subplots()
            if args.plot_train:
                ax3.plot(range(len(train_cr_smooth)), train_cr_smooth)
                ax3_legends.append(models[i])
            if args.plot_val:
                ax3.plot(val_episode, val_cr)
                ax3_legends.append(models[i])

            ax3.legend(ax3_legends)
            ax3.set_xlabel('Number of episodes')
            ax3.set_ylabel('Collision Rate')
            ax3.set_title('')
            plt.grid(True)
            plt.ylim([0, 0.6])


        # plot reward
        if args.plot_reward:
            if ax4 is None:
                _, ax4 = plt.subplots()
                ax4_legends.append("Training")
            if args.plot_train:
                ax4.plot(range(len(train_reward_smooth)), train_reward_smooth)
                #ax4_legends.append(models[i])
            if args.plot_val:
                ax4.plot(val_episode, val_reward)
                ax4_legends.append("Validation")

            ax4.legend(ax4_legends)
            ax4.set_xlabel('Number of episodes')
            ax4.set_ylabel('Reward')
            ax4.set_title('SARL*')
            plt.grid(True)
            plt.ylim([0, 5]) 

    plt.show()


if __name__ == '__main__':
    main()
