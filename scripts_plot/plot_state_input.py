from matplotlib_starter import MatplotlibStarter, MetaData
import sys, signal

signal.signal(signal.SIGINT, lambda signal, frame: sys.exit(0))

with_robust = True
if with_robust:
    ax1 = [MetaData("log/diabolo_state_input_with_robust.log", [-1, 2], label="Pitch", color="blue", lambda_function=[lambda x:x/100., lambda x:x], start=33, end=48),
           MetaData("log/diabolo_state_input_with_robust.log", [-1, 3], label="Yaw", color="green", lambda_function=[lambda x:x/100., lambda x:x], start=32, end=48)]
    ax2 = [MetaData("log/diabolo_state_input_with_robust.log", [-1, 0], label="u1", color="purple", lambda_function=[lambda x:x/100., lambda x:x-0.7], start=32, end=48),
           MetaData("log/diabolo_state_input_with_robust.log", [-1, 1], label="u2", color="orange", lambda_function=[lambda x:x/100., lambda x:x*20], start=32, end=48)]
else:
    ax1 = [MetaData("log/diabolo_state_input.log", [-1, 2], label="Pitch", color="blue", lambda_function=[lambda x:x/100., lambda x:x]),
           MetaData("log/diabolo_state_input.log", [-1, 3], label="Yaw", color="green", lambda_function=[lambda x:x/100., lambda x:x])]
    ax2 = [MetaData("log/diabolo_state_input.log", [-1, 0], label="u1", color="purple", lambda_function=[lambda x:x/100., lambda x:x-0.7]),
           MetaData("log/diabolo_state_input.log", [-1, 1], label="u2", color="orange", lambda_function=[lambda x:x/100., lambda x:x*20])]

ax1_title = ["State of Diabolo", "time[s]", "angle[degree]"]
ax2_title = ["Control Input from Robot", "time[s]", ""]

plotter = MatplotlibStarter(hspace=0.6)
plotter.execute(2, 1,
                [[ax1],
                 [ax2]],
                [[ax1_title],
                 [ax2_title]])

