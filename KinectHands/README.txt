To compile and run:

- Make sure the PRenderer2 root directory is in the SAME directory as the 
  KinectHands root.  Otherwise you can fix all the jtil linker dependancies
  manually.
- In Control Panel -> System -> Advanced System Settings -> Environment
  Variables.   Add a new User Variable: name = 'JLIB_DIR', 
  value = 'C:\path\to\prenderer' (whichever directory has jtil_resource_files)
  Alternatively, you can copy jtil_resource_files to the executable directory.

It's better to compile OpenNI sources from scratch (bug fixes etc): to do so
go to the github page, and download it.  Then Open the .sln file.  Then add
$(WindowsSdkDir)include; to the C header include path for all the projects.
Switch to release and build it.