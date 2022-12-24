# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct C:\Users\ihsan\workspace\SVM_Speech_Platform\platform.tcl
# 
# OR launch xsct and run below command.
# source C:\Users\ihsan\workspace\SVM_Speech_Platform\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {SVM_Speech_Platform}\
-hw {C:\Users\ihsan\Documents\Vivado_project\SVM_Speech_30\design_1_wrapper.xsa}\
-proc {ps7_cortexa9_0} -os {standalone} -out {C:/Users/ihsan/workspace}

platform write
platform generate -domains 
platform active {SVM_Speech_Platform}
platform generate
catch {platform remove uarthello_platform}
platform generate
