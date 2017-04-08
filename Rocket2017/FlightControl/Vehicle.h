#define WET_MASS        5.0     // kilograms

#define DRY_MASS        3.75    // kilograms

#define ATM_DENSITY     1.225   // kg/m^3

#define REF_AREA_BODY   0.0081  // m^2

#define REF_AREA_FLAP   0.0016  // m^2

#define CD_PASSIVE      0.5     // unitless

#define CD_ACTIVE       0.6     // unitless

#define TARGET_ALT      3500    // meters

#define K_PASSIVE       (0.5 * ATM_DENSITY * CD_PASSIVE * REF_AREA_BODY)

#define K_ACTIVE        (0.5 * ATM_DENSITY * CD_ACTIVE * (REF_AREA_BODY + 4*REF_AREA_FLAP))
