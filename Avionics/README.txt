###################################################################
#                                                                 #
#                     CC BY-NC-SA 4.0 License                     #
#                                                                 #
#   Except where otherwise noted, this code is licensed under a   #
# Attribution-NonCommercial-ShareAlike 4.0 International License. #
#                                                                 #
###################################################################

- Download CircuitPython .UF2 file from the following link: (Only tested in US English)
  https://circuitpython.org/board/raspberry_pi_pico/

- Plug Raspberry in to computer

- If the Pico shows up as RPI-RP2 you can drag the .UF2 files onto the drive

- The drive name should change to CIRCUITPY

- Delete the ENTIRE contents of CIRCUITPY (DO NOT FORMAT THE DRIVE, JUST HIGHTLIGHT AND DELETE THE CONTENTS)

- Replace contents of the CIRCUITPY drive with the contents of Avionics Code

- Open the code.py that should now be on CIRCUITPY with an IDE (Thonny or MU is reccomended)

- Run the code and see if you have any errors

- If errors are popping up, but the code appears to still be functional, you are fine.
  Likely the issue is a missing/corrupt SD card or the Power Board is not connected

- If the code outright fails, this could be a sign of a more serious issue
  The most common failure is a MemoryError or similar
  To solve this, repeat the previous steps, making sure you REPLACED the contents of CIRCUITPY
  This error occurs because the lib folder needs to contain .mpy files, not .py files
  code.py should be the only .py file

- Feel free to contact us or others familar with the project should you have any issues