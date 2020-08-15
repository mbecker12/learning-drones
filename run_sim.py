from subprocess import Popen, PIPE

bot1 = Popen(
    ["/bin/bash", "-e", "python", "-i", "main_loop.py"],
    stdout=PIPE,
    stderr=PIPE,
    stdin=PIPE,
)
bot2 = Popen(
    ["/bin/bash", "-e", "python", "-i", "vtk_window.py"],
    stdout=PIPE,
    stderr=PIPE,
    stdin=PIPE,
)
bot3 = Popen(
    ["/bin/bash", "-e", "python", "-i", "visualization.py"],
    stdout=PIPE,
    stderr=PIPE,
    stdin=PIPE,
)
