## FPGA-based digital PID controller

This repository contains code that, when combined with the appropriate hardware, implements a high-precision FPGA-based digital proportional-integral-derivative controller.  The original purpose of this device was for controller large electric currents which created large magnetic fields for cold-atoms experiments, but it can be adapted for other purposes as well.  As the electric currents were controlled using MOSFETs, which are highly nonlinear, this PID controller implements what is known as _gain-scheduling_ which changes the PID gain coefficients based on the set-point.  This enables the controller to maintain a consistent closed-loop response if the corresponding changes in the open-loop response are known.  See [this paper](https://doi.org/10.1063/1.5128935) for some background and performance measurements.  The arXiv version is [here](https://arxiv.org/abs/1909.11257).

Detailed information about the internal workings of the device are available in the documentation folder.  This readme is intended to get the software up and running.  You should consult the documentation for further information.


## Requirements

This project was built using Xilinx FPGAs and Xilinx tools.  Experienced users can likely port this to other manufacturers.  Software drivers are written in MATLAB, but the documentation folder provides a reference of the different registers so programming drivers in a different language (e.g. Python) should be relatively straightforward.

**Hardware**
- [Numato labs Saturn Spartan 6 FPGA Development Board With DDR SDRAM](https://numato.com/product/saturn-spartan-6-fpga-development-board-with-ddr-sdram), specifically the XC6SLX45 version.
- Analog Devices AD5791 evaluation board
- Texas Instruments ADS127L01 evaluation board
- A computer capable of running Xilinx's ISE 14.7 software

## Installing ISE 14.7

ISE 14.7 is no longer supported, although Xilinx released a version that "runs" on Windows 10.  It does nothing of the sort: the installer installs a Linux virtual machine on the computer and then installs ISE on that virtual machine.  I tried to use this once and it didn't go that well, so I might suggest that you go through the steps manually.

First, you will need a computer (real or virtual) that is running Linux Debian 10 (Ubuntu 18.04 will likely work exactly the same).  If you want to run a virtual instance, I suggest Oracle VirtualBox - there are some good tutorials online on how to install a virtual machine running Debian 10 on your computer.  You will need to allocate a large amount of space for the virtual drive, though: I suggest 96-128 GB.  

### Download installation file

Once you have Debian 10 running, you will need to download the installation file.  For simplicity you should download the full installation file and not the 4-part file.  The download page is [here](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive-ise.html); scroll down to the section titled 'ISE Design Suite - 14.7  Full Product Installation'.  The link you want is the 'Full Installer for Linux'.  Release notes can be found [here](https://www.xilinx.com/support/documentation/sw_manuals/xilinx14_7/irn.pdf): note that the link on Xilinx's website is broken, as it has an extra '.html' appended to the file name.  You will need a Xilinx account to download the file, and you will need to fill out an export form because the US government is very silly.  The file is large - 6.09 GB - so the download will take a while.

This downloads a tar archive, so you will need to unpack it somewhere.  Create a directory in your home directory called Xilinx ('mkdir ~/Xilinx'), navigate to the Downloads directory, and extract the tar archive in the downloads folder to this directory using 

    tar -C ~/Xilinx/ -xvf Xilinx_ISE_DS_Lin_14.7_1015_1.tar

This assumes that the .tar file name is 'Xilinx_ISE_DS_Lin_14.7_1015_1.tar'.

### Run installation file

Next, you will need to run the installation file.  Move into the new directory in the '~/Xilinx' folder, and run the script 'xsetup' using './xsetup'.  This may throw an error where it claims that the system object 'libncurses.so.5' cannot be found.  This is probably because Debian 10 is either using a newer version or isn't using this library.  You will need to install the libncurses library.  To do this, use the 'apt-get install' command:

    sudo apt-get install libncurses5

If there are other system objects that cannot be found, you will need to install the correct versions in the same way.

Once libncurses5 is installed, re-run the installation script again.  This will start a GUI that will go through the installation process.  You will want to install the WebPACK edition when prompted, and you will also want to uncheck the 'Install cable drivers' option.  Proceed through the rest of the installation.

You will also need to get a license file.  If the license manager can't do this automatically, or has trouble launching a web-browser, you can do this manually through the Xilinx website.  Just google Xilinx licensing and get the appropriate license.  Managing licenses can be done through ISE under the 'Help' menu.

### Check that ISE works

Some environment variables need to be set in order for ISE and other tools to run properly.  This is best done automatically every time you open a new terminal.  Open your bash configuration file using 'nano ~/.bashrc', and at the end of the file add the line

    source /opt/Xilinx/14.7/ISE_DS/settings64.sh > /dev/null

This runs the settings64.sh file (use settings32.sh if using a 32-bit machine), and the '> /dev/null' just suppresses output.  Now the environment variables are set, so you should be able to run ISE by simply running the command 'ise' in the terminal.

In order to use the IP Core Generator, you may need to change which version of Java ISE uses for this.  To do so, navigate to '/opt/Xilinx/14.7/ISE_DS/ISE'.  There, you need to move the 'java' directory to some other name, like 'java.bak'.  This keeps it around in case you need it at some point.  Then, make a symbolic link between the 'java6' directory and the 'java' directory using

    ln -sf java6 java

This way, ISE and the IP Core Generator will use Java 6 while thinking that it is using an older version.

### Install drivers

Refer to the Numato labs website for [driver installation instructions](https://numato.com/product/saturn-spartan-6-fpga-development-board-with-ddr-sdram).

## Creating project files

The git repository contains all the source files needed to create three different versions of the architecture - a single controller version, a dual controller version, and a quad controller version.  An additional version, called 'ip-trap-servo', is for internal use with our IP trap servo and uses a different DAC with additional switches.  First, you will need to clone the git repository - navigate to the parent directory you want to use and run

    git clone https://github.com/kjaergaard-lab/fpga-servo

in the command line.  This will download the git repository and put it in the directory 'fpga-servo'.  Under that directory you will find the directories 'single-servo', 'dual-servo', and 'quad-servo'.  To create the ISE project files, navigate to the desired directory (say, 'single-servo') and run

    xtclsh single-servo.tcl rebuild_project

This will run a TCL script that creates the project and links the sources to the project.  Now, if you run ISE and go to 'Open Project' and navigate to the 'single-servo' directory, you will find an .xise file which you can open with ISE.

Before you can synthesize and implement the design, you will need to regenerate the two IP Cores that are used with the project (dcm1 and K_Multiplier).  On the left-hand pane in ISE, select the part 'xc6slx45-3csg324' under 'Hierarchy' (top left under the default layout).  Under 'Design Utilities', double click on 'Regenerate All Cores'.  This will recreate the necessary files for the aforementioned IP Cores.  Note that the memory interface doesn't need to be regenerated (in fact, definitely **don't** do that) because the VHDL implementation was imported directly.

Once the cores are regenerated, you can run the 'Implement Top Module' process which runs the 'Synthesis' and 'Implement Design' stages.  Once those are done, you can run the 'Generate Programming File' stage which will create a file called topmod.bin that you can upload to the FPGA using either the Windows executable file provided by Numato Labs or xc3sprog.  


## Creating a project from scratch

If you want to create a project from scratch, you can follow these instructions (especially if you use a different board):

1. Open ISE, go to File -> New Project, and specify the location for the project directory and the project's name.  At the bottom of the dialog, set the top-level source to 'HDL'
2. On the next pane (Project Settings), set the Family to 'Spartan6', the Device to either 'XC6SLX16' or 'XC6SLX45' (depending on your board), the package to 'CSG324', and the speed grade to '-3'.  Under Preferred Language, set this to 'VHDL' and the source analysis standard to 'VHDL-93'.
3. On the next page, click 'Finish'.

    You should now have a new directory under your project name.

4. Under the new project directory, create a directory called 'sources'.  Copy all VHDL files and the .ucf file into this directory.
5. We will now need to regenerate all the IP components for clocking, multiplication, and the memory interface.  Under 'Tools', open the 'Core Generator'.  We'll start with clocking.
    1. Under the Core Generator, go to 'FPGA Features and Design', then 'Clocking', and then 'Clocking Wizard'.  Double click on that.
    2. Under the Clocking Wizard, rename the component to be 'dcm1'.  The default values on this page should be fine, but make sure that the input clock frequency is 100 MHz and a 'single ended clock capable pin'.
    3. On the next page, enable 'CLK_OUT2' and set the output frequency to 50 MHz.  On the next page, disable the 'RESET' and 'LOCKED' pins - we don't need them.
    4. Click through the rest of the pages and click 'Generate'.
6. Our next component is the multiplier
    1. Under the Core Generator, go to 'Math Functions', then 'Multipliers', and double click on 'Multipler'.  
    2. Name the component 'K_Multiplier'.  Set the type to 'parallel' and set port A to be a signed type with a width of 40 bits, and set port B to be unsigned with a width of 16 bits.
    3. On the next page, set the multiplier construction to 'Use Mults' and the optimization options to be 'Area Optimized'
    4. On the next page, set the pipelining stages to 6 - this should also be the optimal value reported by the Core Generator.  Click Generate.
7. The last component is the memory interface.
    1. Under the Core Generator, go to 'Memories & Storage Elements', then 'Memory Interface Generators', and then double click on 'MIG Virtex-6 and Spartan-6'.
    2. The first page of the generator should show the project options.  Make sure that the FPGA family, part and speed grade are all what you set in ISE.  Also make sure that the Design Entry is in VHDL.
    2. On the next page, we want to 'Create Design'.  Call the component 's6_lpddr'
    3. Click next on the next page.
    4. On the page 'Memory Selection', choose Bank 3 to have an LPDDR memory type.
    5. On the next page 'Options for C3-LPDDR', set the clock period to 10000 ps (100 MHz).  Set the part number to be 'MT46H32M16XXXX-5'
    6. Leave everything the same on the next page.
    7. On the page 'Port Configuration', set the it to 'Four 32-bit bi-directional ports'.  We are going to create only a single servo, so disable ports 1 - 3.  Leave the memory address mapping selection on Row-Bank-Column.
    8. Click 'Next' on the next page, 'Arbitration'.
    9. Under the next page 'FPGA options', set the RZQ pin to 'N4', disable the debugging signals, and set the 'system clock' to 'single-ended'
    10. Click through the rest of the pages and generate the output.
    11. In ISE, click 'Add Source' and then navigate to 'ipcore_dir/s6_lpddr/user_design/rtl/' and add all the files in the folder.
    12. The design generated by MIG technically won't work with a clock signal generated by the DCM we created earlier, so we need to modify the VHDL files.  In the file 'memc3_infrastructure', find the _generate_ statement 'se_input_clk'.  You will need to comment out the IBUFG component and add the line 'sys_clk_ibufg <= sys_clk;'.  You will also need to chanel the generic constant 'COMPENSATION' under the component 'u_pll_adv' from 'INTERNAL' to 'DCM2PLL'.
8. Finally, you should add the necessary VHDL sources from the vhdl-common directory and the project-specific sources directory.

    