Some boards are not supported by PlatformIO directly. To add support for them just copy
the boards_sam.txt and variants folder to $HOMEDIR/.platformio/packages

Note to not delete other variants already present. Replacing boards_sam.txt deletes the
old version. If you only had the original due boards so far this will be no problem. If
not merge the board definitions into one new file.
