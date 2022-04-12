import robot_upstart
startup = robot_upstart.Job()
startup.add(package="app_node",filename="launch/startup.launch")
startup.install()


