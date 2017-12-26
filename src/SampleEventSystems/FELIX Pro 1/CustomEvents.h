extern void Felix100MS();
extern void Felix500MS();
extern void FelixContainCoordinates();
extern bool customMCode(GCode *com);
extern void cExecute(int action,bool allowMoves);
extern void cNextPrevious(int action,bool allowMoves,int increment);
extern void cOkWizard(int action);
extern bool cExecuteOverride(int action,bool allowMoves);
extern void cRelaxExtruderEndstop();
extern bool cRefreshPage();
extern bool cCustomParser(char c1, char c2);

#undef EVENT_TIMER_100MS
#undef EVENT_TIMER_500MS
#undef EVENT_CONTRAIN_DESTINATION_COORDINATES
#undef EVENT_UNHANDLED_M_CODE
#undef EVENT_UI_EXECUTE
#undef EVENT_UI_NEXTPREVIOUS
#undef EVENT_UI_OK_WIZARD
#undef EVENT_UI_OVERRIDE_EXECUTE
#undef EVENT_BEFORE_Z_HOME
#undef EVENT_UI_REFRESH_PAGE
#undef EVENT_CUSTOM_TEXT_PARSER

#define EVENT_TIMER_100MS {Felix100MS();}
#define EVENT_TIMER_500MS {Felix500MS();}
#define EVENT_CONTRAIN_DESTINATION_COORDINATES FelixContainCoordinates();
#define EVENT_UNHANDLED_M_CODE(c) customMCode(c)
#define EVENT_UI_EXECUTE(action,allowMoves) cExecute(action,allowMoves)
#define EVENT_UI_OVERRIDE_EXECUTE(action,allowMoves) cExecuteOverride(action,allowMoves)
#define EVENT_UI_NEXTPREVIOUS(action,allowMoves,increment) cNextPrevious(action,allowMoves,increment)
#define EVENT_UI_OK_WIZARD(action)  cOkWizard(action)
#define EVENT_BEFORE_Z_HOME cRelaxExtruderEndstop()
#define EVENT_UI_REFRESH_PAGE cRefreshPage()
#define EVENT_CUSTOM_TEXT_PARSER(c1,c2) cCustomParser(c1,c2)
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

#define UI_ACTION_FC_SELECT1    1530
#define UI_ACTION_FC_SELECT2    1531
#define UI_ACTION_FC_PLA        1532
#define UI_ACTION_FC_PETG       1533
#define UI_ACTION_FC_PVA        1534
#define UI_ACTION_FC_FLEX       1535
#define UI_ACTION_FC_ABS        1536
#define UI_ACTION_FC_GLASS      1537
#define UI_ACTION_FC_WOOD       1538
#define UI_ACTION_FC_CUSTOM     1539
#define UI_ACTION_FC_CUSTOM_SET 1540
#define UI_ACTION_FC_WAITHEAT   1541
#define UI_ACTION_FC_BACK1      1542
#define UI_ACTION_FC_BACK2      1543
#define UI_ACTION_HALFAUTO_LEV  1544
#define UI_ACTION_HALFAUTO_LEV2 1545
#define UI_ACTION_HALFAUTO_LEV3 1546
#define UI_ACTION_CZREFH        1547
#define UI_ACTION_START_CZREFH  1548
#define UI_ACTION_CZREFH_INFO   1549
#define UI_ACTION_CZREFH_SUCC   1550

#ifndef HALF_P1_X
#define HALF_P1_X 25
#define HALF_P1_Y 40
#define HALF_P2_X 25
#define HALF_P2_Y 170
#define HALF_FIX_X 132
#define HALF_FIX_Y 110
#ifdef TEC4
#define HALF_Z 2.8
#else
#define HALF_Z 3
#endif 
// Wheel position
#define HALF_WHEEL_P1 -71
#define HALF_WHEEL_P2 199
#endif
#ifndef HALF_PITCH
#define HALF_PITCH 0.7
#endif

#ifndef ZPROBE_REF_HEIGHT
#define ZPROBE_REF_HEIGHT 0.4
#endif


