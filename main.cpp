//=====[Libraries]=============================================================

#include "mbed.h"
#include "arm_book_lib.h"

//=====[Defines]===============================================================

#define NUMBER_OF_KEYS                           4
#define BLINKING_TIME_GAS_ALARM               1000
#define BLINKING_TIME_OVER_TEMP_ALARM          500
#define BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM  100
#define NUMBER_OF_AVG_SAMPLES                   100
#define OVER_TEMP_LEVEL                         50
#define TIME_INCREMENT_MS                       10
#define DEBOUNCE_KEY_TIME_MS                    40
#define KEYPAD_NUMBER_OF_ROWS                    4
#define KEYPAD_NUMBER_OF_COLS                    4
#define EVENT_MAX_STORAGE                      100
#define EVENT_NAME_MAX_LENGTH                   14

//=====[Declaration of public data types]======================================

/**
* El enum matrixKeypadState_t se utiliza para representar los estados de la
* máquina de estados utilizada para el teclado matricial
*/

typedef enum {
    MATRIX_KEYPAD_SCANNING,
    MATRIX_KEYPAD_DEBOUNCE,
    MATRIX_KEYPAD_KEY_HOLD_PRESSED
} matrixKeypadState_t;

typedef struct systemEvent {
    time_t seconds;
    char typeOfEvent[EVENT_NAME_MAX_LENGTH];
} systemEvent_t;

//=====[Declaration and initialization of public global objects]===============

DigitalIn alarmTestButton(BUTTON1);
DigitalIn mq2(PE_12);

DigitalOut alarmLed(LED1);
DigitalOut incorrectCodeLed(LED3);
DigitalOut systemBlockedLed(LED2);

DigitalInOut sirenPin(PE_10);

UnbufferedSerial uartUsb(USBTX, USBRX, 115200);

AnalogIn lm35(A1);

DigitalOut keypadRowPins[KEYPAD_NUMBER_OF_ROWS] = {PB_3, PB_5, PC_7, PA_15};
DigitalIn keypadColPins[KEYPAD_NUMBER_OF_COLS]  = {PB_12, PB_13, PB_15, PC_6};

//=====[Declaration and initialization of public global variables]=============

bool alarmState    = OFF;
bool incorrectCode = false;
bool overTempDetector = OFF;

int numberOfIncorrectCodes = 0;
int numberOfHashKeyReleasedEvents = 0;
int keyBeingCompared    = 0;
char codeSequence[NUMBER_OF_KEYS]   = { '1', '8', '0', '5' };
char keyPressed[NUMBER_OF_KEYS] = { '0', '0', '0', '0' };
int accumulatedTimeAlarm = 0;

bool alarmLastState        = OFF;
bool gasLastState          = OFF;
bool tempLastState         = OFF;
bool ICLastState           = OFF;
bool SBLastState           = OFF;

bool gasDetectorState          = OFF;
bool overTempDetectorState     = OFF;

float potentiometerReading = 0.0;
float lm35ReadingsAverage  = 0.0;
float lm35ReadingsSum      = 0.0;
float lm35ReadingsArray[NUMBER_OF_AVG_SAMPLES];
float lm35TempC            = 0.0;

int accumulatedDebounceMatrixKeypadTime = 0;
int matrixKeypadCodeIndex = 0;
char matrixKeypadLastKeyPressed = '\0';
char matrixKeypadIndexToCharArray[] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D',
};
int row = 0;
int col = 0;

/**
* La variable matrixKeypaddState del tipo matrixKeypadState_t será la que
* permita utilizar la máquina de estados correspondiente al teclado matricial.
* estadoAnterior es una variable que será utilizada para debugear la máquina
* de estados.
* La variable es global, puede ser leida y modificada por todas las funciones.
*/
matrixKeypadState_t matrixKeypadState;
matrixKeypadState_t estadoAnterior;
char matrixKeypadStateString[3][40] = {"MATRIX_KEYPAD_SCANNING",
    "MATRIX_KEYPAD_DEBOUNCE",
    "MATRIX_KEYPAD_KEY_HOLD_PRESSED"};

int eventsIndex            = 0;
systemEvent_t arrayOfStoredEvents[EVENT_MAX_STORAGE];

//=====[Declarations (prototypes) of public functions]=========================

void inputsInit();
void outputsInit();

void alarmActivationUpdate();
void alarmDeactivationUpdate();

void uartTask();
void availableCommands();
bool areEqual();

void eventLogUpdate();
void systemElementStateUpdate( bool lastState,
                               bool currentState,
                               const char* elementName );

float celsiusToFahrenheit( float tempInCelsiusDegrees );
float analogReadingScaledWithTheLM35Formula( float analogReading );
void lm35ReadingsArrayInit();

void matrixKeypadInit();
char matrixKeypadScan();
char matrixKeypadUpdate();

void debug();

//=====[Main function, the program entry point after power on or reset]========

int main()
{
    inputsInit();
    outputsInit();
    while (true) {
        alarmActivationUpdate();
        alarmDeactivationUpdate();
        uartTask();
        eventLogUpdate();
        delay(TIME_INCREMENT_MS);
        debug();
    }
}

//=====[Implementations of public functions]===================================

void inputsInit()
{
    lm35ReadingsArrayInit();
    alarmTestButton.mode(PullDown);
    sirenPin.mode(OpenDrain);
    sirenPin.input();
    matrixKeypadInit();
    mq2.mode(PullUp);
}

void outputsInit()
{
    alarmLed = OFF;
    incorrectCodeLed = OFF;
    systemBlockedLed = OFF;
}

void alarmActivationUpdate()
{
    static int lm35SampleIndex = 0;
    int i = 0;

    lm35ReadingsArray[lm35SampleIndex] = lm35.read();
    lm35SampleIndex++;
    if ( lm35SampleIndex >= NUMBER_OF_AVG_SAMPLES) {
        lm35SampleIndex = 0;
    }
    
    lm35ReadingsSum = 0.0;
    for (i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsSum = lm35ReadingsSum + lm35ReadingsArray[i];
    }
    lm35ReadingsAverage = lm35ReadingsSum / NUMBER_OF_AVG_SAMPLES;
       lm35TempC = analogReadingScaledWithTheLM35Formula ( lm35ReadingsAverage );    
    
    if ( lm35TempC > OVER_TEMP_LEVEL ) {
        overTempDetector = ON;
    } else {
        overTempDetector = OFF;
    }

    if( !mq2) {
        gasDetectorState = ON;
        alarmState = ON;
    }
    if( overTempDetector ) {
        overTempDetectorState = ON;
        alarmState = ON;
    }
    if( alarmTestButton ) {             
        overTempDetectorState = ON;
        gasDetectorState = ON;
        alarmState = ON;
    }
    if( alarmState ) { 
        accumulatedTimeAlarm = accumulatedTimeAlarm + TIME_INCREMENT_MS;
        sirenPin.output();                                     
        sirenPin = LOW;                                

        if( gasDetectorState && overTempDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM ) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        } else if( gasDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_ALARM ) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        } else if ( overTempDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_OVER_TEMP_ALARM  ) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        }
    } else{
        alarmLed = OFF;
        gasDetectorState = OFF;
        overTempDetectorState = OFF;
        sirenPin.input();                                  
    }
}

void alarmDeactivationUpdate()
{
    if ( numberOfIncorrectCodes < 5 ) {
        /**
        * matrixKeypadUpdate devuelve el caracter leído por el tablero matricial.
        * Si no se leyó ninguno se devuelve un \0. En ambos casos, el valor se guarda
        * en keyReleased.
        */
        char keyReleased = matrixKeypadUpdate();
        if( keyReleased != '\0' && keyReleased != '#' ) {
            keyPressed[matrixKeypadCodeIndex] = keyReleased;
            if( matrixKeypadCodeIndex >= NUMBER_OF_KEYS ) {
                matrixKeypadCodeIndex = 0;
            } else {
                matrixKeypadCodeIndex++;
            }
        }
        /**
        * Si se presiona el # hay dos opciones. Si estaba activado incorrectCodeLed
        * se fija si se presionó dos veces. En ese caso se pone un 0 en incorrectCodeLed.
        * Si está prendida alarmState llama a la función areEqual para revisar si el código
        * introducido es el correcto.
        */
        if( keyReleased == '#' ) {
            if( incorrectCodeLed ) {
                numberOfHashKeyReleasedEvents++;
                if( numberOfHashKeyReleasedEvents >= 2 ) {
                    incorrectCodeLed = OFF;
                    numberOfHashKeyReleasedEvents = 0;
                    matrixKeypadCodeIndex = 0;
                }
            } else {
                if ( alarmState ) {
                    if ( areEqual() ) {
                        alarmState = OFF;
                        numberOfIncorrectCodes = 0;
                        matrixKeypadCodeIndex = 0;
                    } else {
                        incorrectCodeLed = ON;
                        numberOfIncorrectCodes++;
                    }
                }
            }
        }
    } else {
        systemBlockedLed = ON;
    }
}

void uartTask()
{
    char receivedChar = '\0';
    char str[100];
    int stringLength;
    if( uartUsb.readable() ) {
        uartUsb.read( &receivedChar, 1 );
        switch (receivedChar) {
        case '1':
            if ( alarmState ) {
                uartUsb.write( "The alarm is activated\r\n", 24);
            } else {
                uartUsb.write( "The alarm is not activated\r\n", 28);
            }
            break;

        case '2':
            if ( !mq2 ) {
                uartUsb.write( "Gas is being detected\r\n", 22);
            } else {
                uartUsb.write( "Gas is not being detected\r\n", 27);
            }
            break;

        case '3':
            if ( overTempDetector ) {
                uartUsb.write( "Temperature is above the maximum level\r\n", 40);
            } else {
                uartUsb.write( "Temperature is below the maximum level\r\n", 40);
            }
            break;
            
        case '4':
            uartUsb.write( "Please enter the four digits numeric code ", 42 );
            uartUsb.write( "to deactivate the alarm: ", 25 );

            incorrectCode = false;

            for ( keyBeingCompared = 0;
                  keyBeingCompared < NUMBER_OF_KEYS;
                  keyBeingCompared++) {
                uartUsb.read( &receivedChar, 1 );
                uartUsb.write( "*", 1 );
                if ( codeSequence[keyBeingCompared] != receivedChar ) {
                    incorrectCode = true;
                }
            }

            if ( incorrectCode == false ) {
                uartUsb.write( "\r\nThe code is correct\r\n\r\n", 25 );
                alarmState = OFF;
                incorrectCodeLed = OFF;
                numberOfIncorrectCodes = 0;
            } else {
                uartUsb.write( "\r\nThe code is incorrect\r\n\r\n", 27 );
                incorrectCodeLed = ON;
                numberOfIncorrectCodes++;
            }
            break;

        case '5':
            uartUsb.write( "Please enter the new four digits numeric code ", 46 );
            uartUsb.write( "to deactivate the alarm: ", 25 );

            for ( keyBeingCompared = 0;
                  keyBeingCompared < NUMBER_OF_KEYS;
                  keyBeingCompared++) {
                uartUsb.read( &receivedChar, 1 );
                uartUsb.write( "*", 1 );
            }

            uartUsb.write( "\r\nNew code generated\r\n\r\n", 24 );
            break;

        case 'c':
        case 'C':
            sprintf ( str, "Temperature: %.2f \xB0 C\r\n", lm35TempC );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;

        case 'f':
        case 'F':
            sprintf ( str, "Temperature: %.2f \xB0 F\r\n", 
                celsiusToFahrenheit( lm35TempC ) );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;
            
        case 's':
        case 'S':
            struct tm rtcTime;
            int strIndex;
                    
            uartUsb.write( "\r\nType four digits for the current year (YYYY): ", 48 );
            for( strIndex=0; strIndex<4; strIndex++ ) {
                uartUsb.read( &str[strIndex] , 1 );
                uartUsb.write( &str[strIndex] ,1 );
            }
            str[4] = '\0';
            rtcTime.tm_year = atoi(str) - 1900;
            uartUsb.write( "\r\n", 2 );

            uartUsb.write( "Type two digits for the current month (01-12): ", 47 );
            for( strIndex=0; strIndex<2; strIndex++ ) {
                uartUsb.read( &str[strIndex] , 1 );
                uartUsb.write( &str[strIndex] ,1 );
            }
            str[2] = '\0';
            rtcTime.tm_mon  = atoi(str) - 1;
            uartUsb.write( "\r\n", 2 );

            uartUsb.write( "Type two digits for the current day (01-31): ", 45 );
            for( strIndex=0; strIndex<2; strIndex++ ) {
                uartUsb.read( &str[strIndex] , 1 );
                uartUsb.write( &str[strIndex] ,1 );
            }
            str[2] = '\0';
            rtcTime.tm_mday = atoi(str);
            uartUsb.write( "\r\n", 2 );

            uartUsb.write( "Type two digits for the current hour (00-23): ", 46 );
            for( strIndex=0; strIndex<2; strIndex++ ) {
                uartUsb.read( &str[strIndex] , 1 );
                uartUsb.write( &str[strIndex] ,1 );
            }
            str[2] = '\0';
            rtcTime.tm_hour = atoi(str);
            uartUsb.write( "\r\n", 2 );

            uartUsb.write( "Type two digits for the current minutes (00-59): ", 49 );
            for( strIndex=0; strIndex<2; strIndex++ ) {
                uartUsb.read( &str[strIndex] , 1 );
                uartUsb.write( &str[strIndex] ,1 );
            }
            str[2] = '\0';
            rtcTime.tm_min  = atoi(str);
            uartUsb.write( "\r\n", 2 );

            uartUsb.write( "Type two digits for the current seconds (00-59): ", 49 );
            for( strIndex=0; strIndex<2; strIndex++ ) {
                uartUsb.read( &str[strIndex] , 1 );
                uartUsb.write( &str[strIndex] ,1 );
            }
            str[2] = '\0';
            rtcTime.tm_sec  = atoi(str);
            uartUsb.write( "\r\n", 2 );

            rtcTime.tm_isdst = -1;
            set_time( mktime( &rtcTime ) );
            uartUsb.write( "Date and time has been set\r\n", 28 );

            break;
                        
            case 't':
            case 'T':
                time_t epochSeconds;
                epochSeconds = time(NULL);
                sprintf ( str, "Date and Time = %s", ctime(&epochSeconds));
                uartUsb.write( str , strlen(str) );
                uartUsb.write( "\r\n", 2 );
                break;

            case 'e':
            case 'E':
                for (int i = 0; i < eventsIndex; i++) {
                    sprintf ( str, "Event = %s\r\n", 
                        arrayOfStoredEvents[i].typeOfEvent);
                    uartUsb.write( str , strlen(str) );
                    sprintf ( str, "Date and Time = %s\r\n",
                        ctime(&arrayOfStoredEvents[i].seconds));
                    uartUsb.write( str , strlen(str) );
                    uartUsb.write( "\r\n", 2 );
                }
                break;

        default:
            availableCommands();
            break;

        }
    }
}

void availableCommands()
{
    uartUsb.write( "Available commands:\r\n", 21 );
    uartUsb.write( "Press '1' to get the alarm state\r\n", 34 );
    uartUsb.write( "Press '2' to get the gas detector state\r\n", 41 );
    uartUsb.write( "Press '3' to get the over temperature detector state\r\n", 54 );
    uartUsb.write( "Press '4' to enter the code sequence\r\n", 38 );
    uartUsb.write( "Press '5' to enter a new code\r\n", 31 );
    uartUsb.write( "Press 'f' or 'F' to get lm35 reading in Fahrenheit\r\n", 52 );
    uartUsb.write( "Press 'c' or 'C' to get lm35 reading in Celsius\r\n", 49 );
    uartUsb.write( "Press 's' or 'S' to set the date and time\r\n", 43 );
    uartUsb.write( "Press 't' or 'T' to get the date and time\r\n", 43 );
    uartUsb.write( "Press 'e' or 'E' to get the stored events\r\n\r\n", 45 );
}

bool areEqual()
{
    int i;

    for (i = 0; i < NUMBER_OF_KEYS; i++) {
        if (codeSequence[i] != keyPressed[i]) {
            return false;
        }
    }

    return true;
}

void eventLogUpdate()
{
    systemElementStateUpdate( alarmLastState, alarmState, "ALARM" );
    alarmLastState = alarmState;

    systemElementStateUpdate( gasLastState, !mq2, "GAS_DET" );
    gasLastState = !mq2;

    systemElementStateUpdate( tempLastState, overTempDetector, "OVER_TEMP" );
    tempLastState = overTempDetector;

    systemElementStateUpdate( ICLastState, incorrectCodeLed, "LED_IC" );
    ICLastState = incorrectCodeLed;

    systemElementStateUpdate( SBLastState, systemBlockedLed, "LED_SB" );
    SBLastState = systemBlockedLed;
}

void systemElementStateUpdate( bool lastState,
                               bool currentState,
                               const char* elementName )
{
    char eventAndStateStr[EVENT_NAME_MAX_LENGTH] = "";

    if ( lastState != currentState ) {

        strcat( eventAndStateStr, elementName );
        if ( currentState ) {
            strcat( eventAndStateStr, "_ON" );
        } else {
            strcat( eventAndStateStr, "_OFF" );
        }

        arrayOfStoredEvents[eventsIndex].seconds = time(NULL);
        strcpy( arrayOfStoredEvents[eventsIndex].typeOfEvent,eventAndStateStr );
        if ( eventsIndex < EVENT_MAX_STORAGE - 1 ) {
            eventsIndex++;
        } else {
            eventsIndex = 0;
        }

        uartUsb.write( eventAndStateStr , strlen(eventAndStateStr) );
        uartUsb.write( "\r\n", 2 );
    }
}

float analogReadingScaledWithTheLM35Formula( float analogReading )
{
    return ( analogReading * 3.3 / 0.01 );
}

float celsiusToFahrenheit( float tempInCelsiusDegrees )
{
    return ( tempInCelsiusDegrees * 9.0 / 5.0 + 32.0 );
}
void lm35ReadingsArrayInit()
{
    int i;
    for( i=0; i<NUMBER_OF_AVG_SAMPLES ; i++ ) {
        lm35ReadingsArray[i] = 0;
    }
}

void matrixKeypadInit()
{
    /**
    * Cuando se inicializa el teclado matricial, la variable matrixKeypaddState
    * se inicializa en el estado MATRIX_KEYPAD_SCANNING.
    * De este estado, si se detecta un caracter distinto de \0, e puede pasar al estado
    * MATRIX_KEYPAD_DEBOUNCE.
    */
    matrixKeypadState = MATRIX_KEYPAD_SCANNING;
    estadoAnterior = MATRIX_KEYPAD_SCANNING;
    int pinIndex = 0;
    /**
    * Los pines correspondientes a las columnas del teclado son pines de entrada.
    * Son los pines que se leerán y que generarán cambios en la máquina de estados.
    * Esos pines se configuran como PullUp y son PB_12, PB_13, PB_15 y PC_6.
    */
    for( pinIndex=0; pinIndex<KEYPAD_NUMBER_OF_COLS; pinIndex++ ) {
        (keypadColPins[pinIndex]).mode(PullUp);
    }
}

char    matrixKeypadScan()
{   char str[100];
    int stringLength;
    int i = 0;

    for( row=0; row<KEYPAD_NUMBER_OF_ROWS; row++ ) {

        for( i=0; i<KEYPAD_NUMBER_OF_ROWS; i++ ) {
            keypadRowPins[i] = ON;
        }

        keypadRowPins[row] = OFF;

        for( col=0; col<KEYPAD_NUMBER_OF_COLS; col++ ) {
            if( keypadColPins[col] == OFF ) {
                return matrixKeypadIndexToCharArray[row*KEYPAD_NUMBER_OF_ROWS + col];
            }
        }
    }
    return '\0';
}

char matrixKeypadUpdate()
{
    char str[100];
    int stringLength;
    char keyDetected = '\0';
    char keyReleased = '\0';

    /**
    * La máquina de estados se modifica leyendo dos variables:
    *   - keyDetected
    *   - accumulatedDebounceMatrixKeypadTime
    * La primera es una variable local de esta función. La segunda es una variable global
    * pero que se modifica solo en esta función. Por lo tanto, el control de los estados
    * se realiza adentro de matrixKeypadUpdate().
    */

    switch( matrixKeypadState ) {

    case MATRIX_KEYPAD_SCANNING:
        keyDetected = matrixKeypadScan();
        if( keyDetected != '\0' ) {
            matrixKeypadLastKeyPressed = keyDetected;
            accumulatedDebounceMatrixKeypadTime = 0;
            matrixKeypadState = MATRIX_KEYPAD_DEBOUNCE;
        }
        break;

    case MATRIX_KEYPAD_DEBOUNCE:
        if( accumulatedDebounceMatrixKeypadTime >=
            DEBOUNCE_KEY_TIME_MS ) {
            keyDetected = matrixKeypadScan();
            if( keyDetected == matrixKeypadLastKeyPressed ) {
                matrixKeypadState = MATRIX_KEYPAD_KEY_HOLD_PRESSED;
            } else {
                matrixKeypadState = MATRIX_KEYPAD_SCANNING;
            }
        }
        accumulatedDebounceMatrixKeypadTime =
            accumulatedDebounceMatrixKeypadTime + TIME_INCREMENT_MS;
        break;

    case MATRIX_KEYPAD_KEY_HOLD_PRESSED:
        keyDetected = matrixKeypadScan();
        if( keyDetected != matrixKeypadLastKeyPressed ) {
            if( keyDetected == '\0' ) {
                keyReleased = matrixKeypadLastKeyPressed;
            }
            matrixKeypadState = MATRIX_KEYPAD_SCANNING;
            
        }
        break;

    default:
        matrixKeypadInit();
        break;
    }
    return keyReleased;
}

void debug()
{
    char mensaje[100];
    char str[100];
    int stringLength;
    
    if(estadoAnterior != matrixKeypadState){
        sprintf(mensaje , "Se pasó del estado %s al estado %s", matrixKeypadStateString[estadoAnterior],
                                                                    matrixKeypadStateString[matrixKeypadState]);
        sprintf ( str, "%s\r\n", mensaje);
        stringLength = strlen(str);
        uartUsb.write( str, stringLength );
        estadoAnterior = matrixKeypadState;
        if(estadoAnterior == MATRIX_KEYPAD_DEBOUNCE){
            sprintf ( str, "Se leyo la columna %d y la fila %d\r\nCorresponde a la tecla %c\r\n",
                        col,row,matrixKeypadIndexToCharArray[row*KEYPAD_NUMBER_OF_ROWS + col]);
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
        }
    }

    if(estadoAnterior != matrixKeypadState){
    
    }
}