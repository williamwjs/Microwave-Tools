制作团队：
王嘉硕5100309436
王子豪5100309443
侯  强5100309451
殷  俊5100309449
刘驰煌5100719070

软件运行方法：

方法一：使用Matlab软件运行
1. 运行Matlab软件。
2. 找到文件路径，在matlab命令窗口输入main，点回车运行。
3. 在运行出现的界面中选择任意功能使用。

方法二：直接运行exe文件
可以直接点击exe文件运行，但是由于Matlab软件的限制（生成的exe不支持符号集），史密斯圆图模块内的双支节调配无法使用。


MATLAB Compiler

1. Prerequisites for Deployment 

. Verify the MATLAB Compiler Runtime (MCR) is installed and ensure you    
  have installed version 7.17 (R2012a).   

. If the MCR is not installed, do following:
  (1) enter
  
      >>mcrinstaller
      
      at MATLAB prompt. This MCR Installer command displays the 
      location of the MCR Installer.

  (2) run the MCR Installer.

Or download Windows 64bit version of MCR from the MathWorks website:

   http://www.mathworks.com/products/compiler/
   
   
For more information about the MCR and the MCR Installer, see 
揥orking With the MCR?in the MATLAB Compiler User抯 Guide.    


NOTE: You will need administrator rights to run MCRInstaller. 


2. Files to Deploy and Package

Files to package for Standalone 
================================
-Project.exe
-MCRInstaller.exe 
   -include when building component by clicking "Add MCR" link 
    in deploytool
-This readme file 

3. Definitions

For information on deployment terminology, go to 
http://www.mathworks.com/help. Select your product and see 
the Glossary in the User抯 Guide.





