import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pickle
import os


# Path to the pkl file
pkl_file_path = './reward.pkl'

# Function to read the latest data from the pkl file
def read_pkl_file(file_path):
    if os.path.exists(file_path):
        with open(file_path, 'rb') as file:
            data = pickle.load(file)
            return data
    return None

# Initialize the plot
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_ylim(-10, 10)  # Set the y-axis limits based on your data
ax.set_xlim(0, 100)  # Set the x-axis limits based on your data
ax.grid()

xdata, ydata = [], []

# Update function for the animation
def update(frame):
    data = read_pkl_file(pkl_file_path)
    if data is not None:
        xdata.append(len(xdata))  # Use the length of xdata as the x value
        ydata.append(data)  # Use the float data as the y value
        line.set_data(xdata, ydata)
        ax.set_xlim(0, len(xdata))  # Adjust x-axis limit dynamically
        ax.set_ylim(min(ydata) - 1, max(ydata) + 1)  # Adjust y-axis limit dynamically
    return line,

# Create an animation
ani = animation.FuncAnimation(fig, update, blit=True, interval=1000)

# Show the plot
plt.show()