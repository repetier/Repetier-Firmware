#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

class Commands {
public:
    static void executeGCode(GCode *com,byte bufferedCommand);
    static void waitUntilEndOfAllMoves();
    static void printCurrentPosition();
    static void printTemperatures();
    static void setFanSpeed(int speed,bool wait); /// Set fan speed 0..255
    static void homeAxis(bool xaxis,bool yaxis,bool zaxis); /// Home axis
    static void homeXAxis();
    static void homeYAxis();
    static void homeZAxis();
    static void changeFeedrateMultiply(int factorInPercent);
    static void changeFlowateMultiply(int factorInPercent);
};

#endif // COMMANDS_H_INCLUDED
