Net F/T C Sample

The Net F/T C Sample is a simple demo showing how to communicate with a Net F/T
over UDP using standard C functions.

The sample was tested using gcc version 3.4.4 running under Cygwin on a 64-bit
Windows 7 computer. 

------------------------------------------------------------
HISTORY
------------------------------------------------------------
Version 1.0 February 7, 2011:
Initial version.

------------------------------------------------------------
FILES
------------------------------------------------------------

netft.c:
A simple C program showing how to request a force/torque sample from a Net F/T
and how to parse the response.

makefile:
A GNU makefile compatible with the 'make' command that creates an executable
'netft' from the netft.c file.

readme.txt:
This readme file.

------------------------------------------------------------
COMPILING USING GCC
------------------------------------------------------------
Place the makefile and netft.c files in the same directory.  Navigate to that
directory on the command line.  Enter the 'make' command.

------------------------------------------------------------
USING THE SAMPLE
------------------------------------------------------------
Enter the command "netft IPADDRESS" at the command line, where the IPADDRESS
token is replaced by the IP address of your Net F/T, e.g. "192.168.1.1", 
without the quotes.  If the IP address is incorrect, or the Net F/T does not
respond to the UDP request, the sample will appear to freeze.  In this case,
press Ctrl-C to terminate the program, and check that you have entered the
correct IP address and that the Net F/T is set up to allow UDP communication.
You can enable UDP communication on the "Communication" page of the Net F/T's
integrated web server.  Also make sure that your computer does not have a
software or hardware firewall that is preventing you from accessing UDP port
49152 on the Net F/T.
