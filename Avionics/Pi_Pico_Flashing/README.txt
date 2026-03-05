Bootloading on the raspberry pi pico is quite simple, but very easy to get wrong so ensure you follow the steps very exactly

1. Loading Micro Python from a factory-reset board
    Simply dowload the .zip of this repository off of github
    Unpack the zip into downloads or a chosen folder
    Plug the Pico into your computer, a file should automatically open up with the path of the pico
    If it doesent, it should show up in file explorer as any other USB drive would
    Drag the .UTF file onto the Pico
    The file should close, the Pico will turn off, and back on again, you may need to unplug and replug in the Pico for the board to reconnect
    Micropython is now loaded onto the board and no further issues should be had

2. Loading the correct version onto a board pre-loaded with the wrong version
    Simply dowload the .zip of this repository off of github
    Unpack the zip into downloads or a chosen folder
    While holding the "Bootsel" button on the Pico, plug the Pico into your computer to enter Bootsel mode
    When the file pops up, drag in the file named "Flash_Nuke"
    This file will effectively factory reset your Pico and you can follow the steps listed above for installing the correct version
    
3. Loading code/software onto the Pico:
    Delete any files that are pre-installed with micropython onto the hard-drive of the pico
    These may look similar to what eventaually ends up on there but just delete them
    Go into the exemplar-cubesat-files folder and drag every file from it into the pico
    Rename fseventd to .fseventd and _code to ._code (github doesent like uploading files with . at the start)
    Hit CTRL S for good luck, unplug replug in the Pico and it should be functional

IMPORTANT: USE THE EXACT FILE INCLUDED IN THIS REPOSITORY OR VERSION:8.0.3 NO OTHER VERSION OF THE BOOT LOADER WILL FUNCITON WITH THE REST OF THE CODEBASE!!!!
EVEN THE .UTF FROM THE PI-PICO SITE ITSELF WILL DOWNLOAD AN INCOMPATABLE VERSION
