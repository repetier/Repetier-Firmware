extern void Felix100MS();
extern void Felix500MS();
extern void FelixContainCoordinates();
extern bool customMCode(GCode *com);
extern void cExecute(int action,bool allowMoves);
extern void cNextPrevious(int action,bool allowMoves,int increment);
extern void cOkWizard(int action);

#undef EVENT_TIMER_100MS
#undef EVENT_TIMER_500MS
#undef EVENT_CONTRAIN_DESTINATION_COORDINATES
#undef EVENT_UNHANDLED_M_CODE
#undef EVENT_UI_EXECUTE
#undef EVENT_UI_NEXTPREVIOUS
#undef EVENT_UI_OK_WIZARD

#define EVENT_TIMER_100MS {Felix100MS();}
#define EVENT_TIMER_500MS {Felix500MS();}
#define EVENT_CONTRAIN_DESTINATION_COORDINATES FelixContainCoordinates();
#define EVENT_UNHANDLED_M_CODE(c) customMCode(c)
#define EVENT_UI_EXECUTE(action,allowMoves) cExecute(action,allowMoves)
#define EVENT_UI_NEXTPREVIOUS(action,allowMoves,increment) cNextPrevious(action,allowMoves,increment)
#define EVENT_UI_OK_WIZARD(action)  cOkWizard(action)

// New menu actions

#define UI_ACTION_XY1_BACK     1500
#define UI_ACTION_XY1_CONT     1501
#define UI_ACTION_XY2_BACK     1502
#define UI_ACTION_XY2_CONT     1503
#define UI_ACTION_EXY_XOFFSET  1504
#define UI_ACTION_EXY_YOFFSET  1505
#define UI_ACTION_EXY_STORE    1506
#define UI_ACTION_CALEX        1507
#define UI_ACTION_CALEX_XY     1508
#define UI_ACTION_CALEX_Z      1509
#define UI_ACTION_CALEX_Z2     1510
#define UI_ACTION_CALEX_Z3     1511
#define UI_ACTION_CALEX_XY3    1512
#define UI_ACTION_PRECOOL1     1513
#define UI_ACTION_PRECOOL2     1514
#define UI_ACTION_REMOVEBED    1515
#define UI_ACTION_SPH_PLA_ACTIVE   1516
#define UI_ACTION_SPH_PETG_ACTIVE  1517
#define UI_ACTION_SPH_PVA_ACTIVE   1518
#define UI_ACTION_SPH_FLEX_ACTIVE  1519
#define UI_ACTION_SPH_ABS_ACTIVE   1520
#define UI_ACTION_SPH_GLASS_ACTIVE 1521
#define UI_ACTION_SPH_WOOD_ACTIVE  1522
#define UI_ACTION_SPH_PLA_ALL   1523
#define UI_ACTION_SPH_PETG_ALL  1524
#define UI_ACTION_SPH_PVA_ALL   1525
#define UI_ACTION_SPH_FLEX_ALL  1526
#define UI_ACTION_SPH_ABS_ALL   1527
#define UI_ACTION_SPH_GLASS_ALL 1528
#define UI_ACTION_SPH_WOOD_ALL  1529
