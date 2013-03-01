To compile and run:

- Fix KinectHands -> Config Prop -> Linker -> Additional Library Directionaries
  (you need to fix the path to jtil.lib and jtil_d.lib)
- In Control Panel -> System -> Advanced System Settings -> Environment
  Variables.   Add a new User Variable: name = 'JLIB_DIR', 
  value = 'C:\path\to\prenderer' (whichever directory has jtil_resource_files)
  Alternatively, you can copy jtil_resource_files to the executable directory.
