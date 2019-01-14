from matplotlib_starter import MatplotlibStarter, MetaData
import sys, signal

signal.signal(signal.SIGINT, lambda signal, frame: sys.exit(0))

ax1 = [MetaData("log/lpf.log", [-1, 0], label="raw Pitch", color="blue", lambda_function=[lambda x:x/100., lambda x:x]),
       MetaData("log/lpf.log", [-1, 1], label="smoothed Pitch", color="green", lambda_function=[lambda x:x/100., lambda x:x])]
ax2 = [MetaData("log/lpf.log", [-1, 2], label="raw Yaw", color="blue", lambda_function=[lambda x:x/100., lambda x:x]),
       MetaData("log/lpf.log", [-1, 3], label="smoothed Yaw", color="green", lambda_function=[lambda x:x/100., lambda x:x])]

ax1_title = ["Pitch", "time[s]", "angle[degree]"]
ax2_title = ["Yaw", "time[s]", "angle[degree]"]

plotter = MatplotlibStarter(hspace=0.6)
plotter.execute(2, 1,
                [[ax1],
                 [ax2]],
                [[ax1_title],
                 [ax2_title]])

