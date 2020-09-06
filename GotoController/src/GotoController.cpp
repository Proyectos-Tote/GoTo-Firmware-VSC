#define _GOTO_CONTROLLER_LEVEL_1_DEBUG_ON_  // Descomentar para habilitar depuración por Serial.
//#define _GOTO_CONTROLLER_LEVEL_2_DEBUG_ON_  // Descomentar para habilitar depuración por Serial.

// Macros para facilitar la salida de información por el Serial.
#ifdef _GOTO_CONTROLLER_LEVEL_1_DEBUG_ON_
#define _GOTO_CONTROLLER_LEVEL_1_DEBUG_(type, text) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.println(text);
#define _GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value);
#define _GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_DOUBLE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value, 15);
#else
#define _GOTO_CONTROLLER_LEVEL_1_DEBUG_(type, text) void();
#define _GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(type, text, value) void();
#define _GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_DOUBLE_(type, text, value) void();
#endif

// Macros para facilitar la salida de información por el Serial.
#ifdef _GOTO_CONTROLLER_LEVEL_2_DEBUG_ON_
#define _GOTO_CONTROLLER_LEVEL_2_DEBUG_(type, text) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.println(text);
#define _GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value);
#define _GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(type, text, value) Serial.print("("); Serial.print(millis()); Serial.print(" millis)"); Serial.print(" ["); Serial.print(type); Serial.print("] "); Serial.print(text); Serial.println(value, 15);
#else
#define _GOTO_CONTROLLER_LEVEL_2_DEBUG_(type, text) void();
#define _GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_(type, text, value) void();
#define _GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(type, text, value) void();
#endif

#include <SPIFFS.h>	
#define FORMAT_SPIFFS_IF_FAILED true

// Clase de control del telescopio.
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteTelesCtrlESPNOW\ToteTelesCtrlESPNOW_V1_1.cpp"

/**************************************************************************************************
 * MUY IMPORTANTE, COMPROBAR LOS DOS DEFINES QUE ESTÁN EN ToteESPNOWDeviceMode.h DE LA CLASE BASE *
 * (ToteESPNOW) ANTES DE COMPILAR                                                                 *
 **************************************************************************************************/

#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteESP32SPIFFSDatabase\ToteESP32SPIFFSDatabase_V1_0.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteAstro\ToteAstro_V1_1.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteTelesJoystick\ToteTelesJoystick.cpp"
#include "C:\Users\Antonio\OneDrive\MIS COSAS\Proyectos Electronica\Arduino\Mis_Librerias\ToteDebouncedBtn\ToteDebouncedBtn.cpp"


#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>


// Prototipos de funciones.
void processMenu(void);			// Comprueba si se eligen opciones del menú.
void printMenu(void);			// Muestra el menú.
void sendHeartbeat(void);		// Envía latido al slave.
void sendINA3221Sense(void);	// Pide información de consumo de corriente en los motores.
void sendARDECRequest(void);	// Pedimos AR y DEC del telescopio.
void repaintMenu(void);			// Redibujo el menú.
void getStatusInfo(void);		// Solicita información de funcionamiento de los motores.
void checkNewMsg(void);			// Compruebo si hay mensajes por atender.
void sendJoystickCmd(void);		// Lee el joystick y envía comando de movimiento a motores.
void sendButtonsCmd(void);		// Lee los botones y envía comandos.
void swapGotoRef(void);			// Intercambio los objetos ref y goto.
void printCurrent(void);		// Muestra por Serial info de consumos.
void printARDECTeles(void);		// Muestra AR y DEC del telescopio.

   
// Pines del ESP32 para controlar el reloj de tiempo real.
#define RTC_DS3231_SDA			21  	// Reservado para que lo use la clase RTC_DS3231.
#define RTC_DS3231_SCL			22		// Reservado para que lo use la clase RTC_DS3231.

// Estas variables están definidas en 'ToteESPNOW'. Son globales para facilitar el acceso a estas por las funciones de callback.
extern DataStruct sendingData;     		// Almacena datos que serán enviados al peer.
extern DataStruct* receivedDataPtr;     // Almacena datos recibidos desde el peer.

// Pines del ESP32 conectados al Joystick.
#define JOYSTICK_AR_PIN		35
#define JOYSTICK_DEC_PIN	34
#define JOYSTICK_SW_PIN		32

// Pines del ESP32 conectados a los botones.
#define SINCRO_AR_DEC_BTN	18
#define LASER_ON_OFF_BTN	19
#define TRACKING_ON_OFF_BTN	2
#define SWAP_GOTO_REF_BTN	23


#define HEARTBEAT_PERIOD			30000	// Periodo entre latidos en milisegundos.
#define GET_STATUS_INFO_PERIOD 		1000	// Periodo de consulta de estado de motores.
#define REPAINT_MENU_PERIOD			1000	// Periodo para repintar el menú.


// Estructura para almacenar lecturas de canales del INA3221
struct INA3221_DATA {
	float busVoltage;
  	float current_mA;
};

// Esta variable almacenará las lecturas de los tres canales del INA3211.
INA3221_DATA ina3221Data[3]; 

// Periodo de sondeo del INA3221
#define INA3221_SENSE_PERIOD		1000	// Se sondea cada segundo.

// Indica si el láser está activo o apagado.
bool laserOnOffToogle = false;				

unsigned long lastHBMillis;				// Almacena millis para calcular la frecuencia de envío de latidos.	
unsigned long lastGSIMillis;			// Almacena millis para calcular la frecuencia de sondeo de estado de los motores.
unsigned long lastINASenseMillis;		// Última vez que se sondeó el INA3221.
unsigned long lastARDECRequestMillis;	// última vez que se pidió la AR y DEC del telescopio.


// Instancio objeto para leer el Joystick analógico. El muestreo es cada 500 ms.
ToteTelesJoystick myJoystick = ToteTelesJoystick(JOYSTICK_AR_PIN, JOYSTICK_DEC_PIN, JOYSTICK_SW_PIN, 500, NULL); 

// Instancio objetos para leer los botones.
ToteDebouncedBtn  mySincroARDECBtn = ToteDebouncedBtn(SINCRO_AR_DEC_BTN, NULL); 	// Lee botón de sincronización de ejes de AR y DEC.	
ToteDebouncedBtn  myLaserONOFFBtn = ToteDebouncedBtn(LASER_ON_OFF_BTN, NULL); 		// Lee botón para encender o apagar el láser.
ToteDebouncedBtn  myTrackingONOFFBtn = ToteDebouncedBtn(TRACKING_ON_OFF_BTN, NULL); // Lee botón para activar o desactivar el tracking del motor de AR.
ToteDebouncedBtn  mySwapGotoRefBtn = ToteDebouncedBtn(SWAP_GOTO_REF_BTN, NULL); 	// Lee botón para activar el swap de goto y Ref.

// Periodo de sondeo de la AR y DEC a la que apunta el telescopio.
#define AR_DEC_REQUEST_PERIOD 	5000	// Se pide cada segundo.

// Instancio objeto para realizar cálculos astronómicos.
ToteAstro myAstro = ToteAstro(false);

// Instancio objeto para administrar la base de datos sqlite3.
ToteESP32SPIFFSDatabase myDB = ToteESP32SPIFFSDatabase();

// Instancio el objeto de control del telescopio.
ToteTelesCtrlESPNOW myTeles = ToteTelesCtrlESPNOW();

// Variables de estado.
bool isARMotorRunning 	= false;		// Semáforo para indicar si se está moviendo el motor de AR.
bool isARMotorSynced 	= false;		// Semáforo para indicar si se AR está sincronizado.
bool isARTracking		= false;		// Semáforo para indicar si se está haciendo tracking.
bool isDECMotorRunning 	= false;		// Semáforo para indicar si se está moviendo el motor de AR.
bool isDECMotorSynced 	= false;		// Semáforo para indicar si se DEC está sincronizado.
bool isComputeAries		= true;			// Semáforo para indicar si se tiene en cuenta el movimiento de la Tierra mientras se hace el Goto.
uint8_t ARDECMotorsVmArrayIndex;		// Almacena índice del roster de velocidad para AR y DEC.

// Variables para enviar comandos de movimiento por el joystick.
bool isStopCmdSent		= false;			// Semáforo que indica que se ha parado los motores.

// Simulará con un menú la interfaz de usuario de ESPNOW.
uint16_t menuSelection = 0; // Por defecto estamos en el menú principal.
uint16_t previousMenuSelection = 0; // Menú del que se viene.
bool isInputBayerConsSelected = false;			// Se pone a 'true' cuando queremos capturar el nombre de la constelación.
bool isInputBayerLetterSelected = false;		// Se pone a 'true' cuando queremos capturar la letra bayer.
bool isInputFlamsteedConsSelected = false;		// Se pone a 'true' cuando queremos capturar la constelación Flamsteed.
bool isInputFlamsteedNumberSelected = false;	// Se pone a 'true' cuando queremos capturar el número Flamsteed.
bool isInputNGCcatIdSelected = false;			// Se pone a 'true' cuando queremos capturar el número Messier.
bool isInputMessierCatIdSelected = false;		// Se pone a 'true' cuando queremos capturar el número Messier.
bool isInputNameSelected = false;				// Se pone a 'true' cuando queremos capturar el nombre.	

//==============
void setup() {
    Serial.begin(115200);

	_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("setup()"), F("configurando setup para CONTROLLER."));

	myAstro.init();  // Inicializamos los cálculos astronómicos.

	// Configuramos botones.
	mySincroARDECBtn.init(INPUT_PULLDOWN); 		// MUY IMPORTANTE. Poner PULL_DOWN o una resistencia PULL_DOWN en el circuito.
	myLaserONOFFBtn.init(INPUT_PULLDOWN);		// MUY IMPORTANTE. Poner PULL_DOWN o una resistencia PULL_DOWN en el circuito.
	myTrackingONOFFBtn.init(INPUT_PULLDOWN);	// MUY IMPORTANTE. Poner PULL_DOWN o una resistencia PULL_DOWN en el circuito.
	mySwapGotoRefBtn.init(INPUT_PULLDOWN);		// MUY IMPORTANTE. Poner PULL_DOWN o una resistencia PULL_DOWN en el circuito.
	
  
	myDB.init(); // Inicializamos la base de datos.
	myDB.openDB("/spiffs/objects.db");  // Abre la base de datos. Ojo con la ruta. Debe estar dentro de /spiffs/
	
	//myDB.format(); // Formateamos.
	myDB.listFS(); // Listamos el sistema de archivos SPIFFS.


    // Inicializo la comunicación ESP-NOW
    myTeles.init();

     // Compruebo resultado
    if (myTeles.getESPNOWStatus()) {
        _GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("setup()"), F("'getESPNOWStatus' devuelve 'true'."));
    } else {
        _GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("setup()"), F("ERROR FATAL. 'getESPNOWStatus' devuelve 'false'."));
    }

	// Mostramos el menú principal
	printMenu();
}

//==============

void loop() {
    // Proceso el menú
	processMenu();

	// Envío Heartbeat.
	sendHeartbeat();

	// Pido sensado de corriente de motores.
	sendINA3221Sense();

	// Pido valor de AR y DEC del telescopio.
	sendARDECRequest();

	// Solicita información de movimiento de motores.
    getStatusInfo();

    // Compruebo si hay mensajes por atender.
    checkNewMsg();

	// Leo el Joystick y envio comandos.
	sendJoystickCmd();

	// Leo botones y envío comandos.
	sendButtonsCmd();
}

// Procesa la entrada de usuario.
void processMenu(void) {
	static String data;
	static char cmd;
	static ToteAstro::CELESTIAL_OBJECT celestialObject;

	if (Serial.available()) {
		//data = Serial.readStringUntil('\n');
		data = Serial.readStringUntil('\n');

		// Comprobación del semáforo de captura de nombre de constelación.
		if (isInputBayerConsSelected) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(F("processMenu()"), F("Constelación elegida: "), data);

			// Almaceno información en campo.
			celestialObject.cons = data;

			// Anulo el semáforo de captura.
			isInputBayerConsSelected = false;

			printMenu();
		}

		// Comprobación del semáforo de captura de nombre de constelación.
		if (isInputBayerLetterSelected) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(F("processMenu()"), F("letra elegida: "), data);

			// Almaceno información en campo.
			celestialObject.bayer = data;

			// Anulo el semáforo de captura.
			isInputBayerLetterSelected = false;

			// Busco el recorset del objeto.
			myDB.selectBayerObject(celestialObject.cons, celestialObject.bayer);

			// Actualizo datos del Goto (Cojo 1er registro del recordset.)
			myAstro.setGotoObject(myDB.getRow(1));

			printMenu();
		}

		// Comprobación del semáforo de captura de nombre de constelación.
		if (isInputFlamsteedConsSelected) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(F("processMenu()"), F("Constelación elegida: "), data);

			// Almaceno información en campo.
			celestialObject.cons = data;

			// Anulo el semáforo de captura.
			isInputFlamsteedConsSelected = false;

			printMenu();
		}

		// Comprobación del semáforo de captura de nombre de constelación.
		if (isInputFlamsteedNumberSelected) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(F("processMenu()"), F("Flamsteed id elegido: "), data);

			// Almaceno información en campo.
			celestialObject.flam = data;

			// Anulo el semáforo de captura.
			isInputFlamsteedNumberSelected = false;

			// Busco el recorset del objeto.
			myDB.selectFlamsteedObject(celestialObject.cons, celestialObject.flam);

			// Actualizo datos del Goto (Cojo 1er registro del recordset.)
			myAstro.setGotoObject(myDB.getRow(1));

			printMenu();
		}

		// Comprobación del semáforo de captura de nombre de constelación.
		if (isInputNGCcatIdSelected) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(F("processMenu()"), F("NGC id: "), data);

			// Almaceno información en campo.
			celestialObject.catId = data;

			// Anulo el semáforo de captura.
			isInputNGCcatIdSelected = false;

			// Busco el recorset del objeto.
			myDB.selectNGCObject(celestialObject.catId);

			// Actualizo datos del Goto (Cojo 1er registro del recordset.)
			myAstro.setGotoObject(myDB.getRow(1));

			printMenu();
		}

		// Comprobación del semáforo de captura de nombre de constelación.
		if (isInputMessierCatIdSelected) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(F("processMenu()"), F("Objeto Messier elegido : "), data);

			// Almaceno información en campo.
			celestialObject.catId = data;

			// Anulo el semáforo de captura.
			isInputMessierCatIdSelected = false;

			// Busco el recorset del objeto.
			myDB.selectMessierObject(celestialObject.catId);

			// Actualizo datos del Goto (Cojo 1er registro del recordset.)
			myAstro.setGotoObject(myDB.getRow(1));

			printMenu();
		}

		// Comprobación del semáforo de captura de nombre común del objeto.
		if (isInputNameSelected) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_VALUE_(F("processMenu()"), F("Objeto elegido : "), data);

			// Almaceno información en campo.
			celestialObject.name = data;

			// Anulo el semáforo de captura.
			isInputNameSelected = false;

			// Busco el recorset del objeto.
			myDB.selectNamedObject(celestialObject.name);

			// Actualizo datos del Goto (Cojo 1er registro del recordset.)
			myAstro.setGotoObject(myDB.getRow(1));

			printMenu();
		}

 		// Convierto la pulsación de teclas en un comando para el telescopio.
		cmd = data.charAt(0);

	
    	// Procesado especial
		if (cmd == 'x') {
			// Solicitamos que el slave detenga inmediatamente los motores.
			sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_HALT;

        	// Enviamos comando.
        	myTeles.sendData();
		}

    
		
		// Procesado especial
		if (cmd == 'm') {
			// Pintamos el menú.
			printMenu();

			// Salimos.
			return;
		}

    	// Procesado especial

		if (cmd == 'r') {
			// Solicitamos que el slave muestre en su serial información sobre los contadores internos.
			sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_PRINT_REPORT;

        	// Enviamos comando.
        	myTeles.sendData();
		}


		// Procesado especial
		if (cmd == 's') {
			// Solicitamos que se detengan los motores de AR y DEC.
			sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_STOP;

        	// Enviamos comando.
        	myTeles.sendData();
		}


		// En función del menú donde estemos
		switch(menuSelection) {
			case 0:	// Estamos en el menú principal.
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '1': // Se eligió MENÚ PRINCIPAL / PONER EN ESTACIÓN. (Identificador de menú = 1)
						menuSelection = 1;
						previousMenuSelection = 0;
						printMenu();
					break;

					case '2': // Se eligió MENÚ PRINCIPAL / SINCRONIZAR AR. (Identificador de menú = 2)
						// Compruebo si hay objeto de refencia establecido.
						if (!myAstro.getIsReferenceObjectSet()) {
							_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("No es posible sincronizar. Hay que establecer un objeto de referencia primero."));

							menuSelection = 0; // Al Menú principal.
							printMenu();

							break;
						}

						menuSelection = 2;
						previousMenuSelection = 0;
						printMenu();
					break;

					case '3': // Se eligió MENÚ PRINCIPAL / SINCRONIZAR DEC. (Identificador de menú = 3)
						// Compruebo si hay objeto de refencia establecido.
						if (!myAstro.getIsReferenceObjectSet()) {
							_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("No es posible sincronizar. Hay que establecer un objeto de referencia primero."));

							menuSelection = 0; // Al Menú principal.
							printMenu();
							break;
						}

						menuSelection = 3;
						previousMenuSelection = 0;
						printMenu();
					break;

					case '4': // Se eligió MENÚ PRINCIPAL / GOTO. (Identificador de menú = 4)
						menuSelection = 4;
						previousMenuSelection = 0;
						printMenu();
					break;

					case '5': // Se eligió MENÚ PRINCIPAL / CALCULAR TRÁNSITO. (Identificador de menú = 5)
						menuSelection = 5;
						previousMenuSelection = 0;
						printMenu();
					break;

					case '6': // Se eligió MENÚ PRINCIPAL / IR A ORIGEN. (Identificador de menú = 6)
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Moviendo a posición de origen."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_GOTO_ORIGIN;
						
						// Enviamos comando.
        				myTeles.sendData();
					
						menuSelection = 0;
						previousMenuSelection = 0;
						printMenu();
					break;

					case '7': // Se eligió MENÚ PRINCIPAL / MOTORES. (Identificador de menú = 7)
						menuSelection = 7;
						previousMenuSelection = 0;
						printMenu();
					break;
				}
			break;  // de case 0:

			case 1:	// Estamos en el MENU PRINCIPAL / PONER EN ESTACIÓN
				previousMenuSelection = 0;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '1': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / FECHA (Identificador de menú = 11)
						myAstro.getUTC();

						// TOTE, TODO: Pedir nueva fecha.
						
						printMenu();
					break;

					case '2': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / HORA (Identificador de menú = 12)
						myAstro.getUTC();
						
						// TOTE, TODO: Pedir nueva fecha.

						printMenu();
					break;

					case '3': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / LATITUD (Identificador de menú = 13)
						menuSelection = 13;
						printMenu();
					break;

					case '4': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / FECHA (Identificador de menú = 14)
						menuSelection = 14;
						printMenu();
					break;

					case '5': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / ESTRELLA DE REFERENCIA (Identificador de menú = 15)
						menuSelection = 15;
						printMenu();
					break;

					case '0': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / VOLVER (Identificador de menú = 0)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 1:

			case 2:	// Estamos en el MENU PRINCIPAL / SINCRONIZAR AR.
				previousMenuSelection = 0;

				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '1': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / +AR
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Posicionamiento manual (INCREMENTAR AR)"));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '2': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / -AR
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Posicionamiento manual (DECREMENTAR AR)"));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP;

        				// Enviamos comando.
        				myTeles.sendData();
						
						printMenu();
					break;

					case '3': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / ROSTER V AR.
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Roster velocidad motor AR/DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTORS_NEXT_ROSTER_VM;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '4': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / STOP AR.
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Deteniendo motor AR."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_STOP;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '5': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / SINCRONIZAR AR
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Sincronizando motor de AR."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_SYNC;

						// Agrego información a la estructura.
						sendingData.utc = myAstro.getUTC();
						sendingData.ra =  myAstro.getReferenceObject().ra;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '6': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / SINCRONIZAR AR / TRACKING ON/OFF 
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Habilitando deteniendo el tracking."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_TRACKING_ON_OFF;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;


					case '0': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / VOLVER (Identificador de menú = 0)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 2:

			case 3:	// Estamos en el MENU PRINCIPAL / SINCRONIZAR DEC.
				previousMenuSelection = 0;

				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '1': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / +DEC
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Posicionamiento manual (INCREMENTAR DEC)"));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP;

        				// Enviamos comando.
        				myTeles.sendData();
					
						printMenu();
					break;

					case '2': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / -DEC
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Posicionamiento manual (DECREMENTAR DEC)"));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '3': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / ROSTER V DEC
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Roster velocidad motor AR/DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTORS_NEXT_ROSTER_VM;

						// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '4': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / STOP DEC.
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Deteniendo motor DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_STOP;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;


					case '5': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / SINCRONIZAR DEC
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Sincronizando motor de DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_SYNC;

						sendingData.dec = myAstro.getReferenceObject().dec;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();		
					break;

					case '0': // Se eligió MENU PRINCIPAL / PONER EN ESTACIÓN / VOLVER (Identificador de menú = 0)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 3:


			case 4:	// Estamos en el MENU PRINCIPAL / GOTO
				previousMenuSelection = 0;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '1': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER.
						menuSelection = 41;  // Menú pedir constelación y letra.			
						printMenu();
					break;

					case '2': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED.
						menuSelection = 42; // Menú pedir constelación y múmero.
						printMenu();
					break;

					case '3': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR OBJECTO NGC.
						isInputNGCcatIdSelected = true; // Habilitado semáforo para capturar string.
						menuSelection = 43; // Menú pedir id.
						printMenu();
					break;

					case '4': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR OBJECTO MESSIER.
						isInputMessierCatIdSelected = true;  // Habilitado semáforo para capturar string.
						menuSelection = 44; // Menú pedir id.
						printMenu();
					break;

					case '5': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR OBJECTO POR NOMBRE.
						isInputNameSelected = true;  // Habilitado semáforo para capturar string.
						menuSelection = 45; // Menú pedir id.
						printMenu();
					break;

					case '6': // Se eligió MENU PRINCIPAL / GOTO / HACER GOTO.
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Haciendo Goto a objeto seleccionado."));

						// Compruebo si se ha elegido un objeto para hacer Goto.
						if (myAstro.getIsGotoObjectSet() == false) {
							_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Hay que elegir un objeto al que hacer Goto."));

							printMenu();

							break;
						}

						// TOTE: TODO. Comprobar si el objeto seleccionado está por encima del horizonte. En caso 
						// negativo cancelar comando

						// Cargo la estructura.
						sendingData.utc = myAstro.getUTC();
						sendingData.ra  = myAstro.getGotoObject().ra;
						sendingData.dec = myAstro.getGotoObject().dec;

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GOTO_OBJECT;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '7':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Cancelando Goto."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_CANCEL_GOTO_OBJECT;

        				// Enviamos comando.
        				myTeles.sendData();

						printMenu();
					break;

					case '8': // Se eligió MENU PRINCIPAL / GOTO / TRACKING ON/OFF 
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Habilitando deteniendo el tracking."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_TRACKING_ON_OFF;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;

					case '9': // Se eligió MENU PRINCIPAL / GOTO / GOTO-->REF 
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Poniendo objeto Goto como estrella de referencia"));

						swapGotoRef();	
					break;

					case 'a': // Se eligió MENU PRINCIPAL / GOTO / ARDEC del telescopio. 
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("visualizando valores de AR y DEC del telescopio"));

						printARDECTeles();	
					break;

					case 'b': // Se eligió MENU PRINCIPAL / GOTO / Compute Aries ON/OFF
						if (isComputeAries) {
							// Conmuto el switch
							isComputeAries = false;

							// Convierto la pulsación de teclas en un comando para el telescopio.
        					sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_COMPUTE_ARIES_OFF;

							_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Desactivando el cálculo del movimiento de la Tierra para el Goto"));
						} else {
							// Conmuto el switch
							isComputeAries = true;

							// Convierto la pulsación de teclas en un comando para el telescopio.
        					sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_COMPUTE_ARIES_ON;

							_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Activando el cálculo del movimiento de la Tierra para el Goto"));
						}
						
						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();	
					break;

					case '0': // Se eligió MENU PRINCIPAL / GOTO / VOLVER (Identificador de menú = 0)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 4:

			case 41:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER
				previousMenuSelection = 4;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '1': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER / CONSTELACION.
						isInputBayerConsSelected = true; // Habilitado semáforo para capturar string.
						menuSelection = 411; // Menú escribir constelación.
						printMenu();
					break;

					case '2': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER / LETRA.
						isInputBayerLetterSelected = true; // Habilitado semáforo para capturar string.
						menuSelection = 412; // Menú escribir letra.
						printMenu();
					break;

					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER / VOLVER (Identificador de menú = 4)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 41:

			case 42:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED
				previousMenuSelection = 4;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '1': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED / CONSTELACION.
						isInputFlamsteedConsSelected = true; // Habilitado semáforo para capturar string.
						menuSelection = 421; // Menú escribir constelación.
						printMenu();
					break;

					case '2': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED / NÚMERO.
						isInputFlamsteedNumberSelected = true; // Habilitado semáforo para capturar string.
						menuSelection = 422; // Menú escribir letra.
						printMenu();
					break;

					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED / VOLVER (Identificador de menú = 4)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 42:

			case 411:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER / CONSTELACIÓN / ESCRIBIR CONSTELACIÓN. 
				previousMenuSelection = 41;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER / CONSTELACIÓN / VOLVER (Identificador de menú = 41)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 411:

			case 412:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER / CONSTELACIÓN / ESCRIBIR LETRA. 
				previousMenuSelection = 41;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA BAYER / LETRA / VOLVER (Identificador de menú = 41)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 412:

			case 421:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED / CONSTELACIÓN / ESCRIBIR CONSTELACIÓN. 
				previousMenuSelection = 42;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEEED / CONSTELACIÓN / VOLVER (Identificador de menú = 42)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 421:

			case 422:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED / NUMERO / ESCRIBIR NÚMERO. 
				previousMenuSelection = 42;
				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR ESTRELLA FLAMSTEED / NÚMERO / VOLVER (Identificador de menú = 42)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 422:

			case 43:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR NGC / ESCRIBIR CATID. 
				previousMenuSelection = 4;

				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR NGC / ESCRIBIR CATID / VOLVER (Identificador de menú = 4)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 43:

			case 44:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR MESSIER / ESCRIBIR CATID. 
				previousMenuSelection = 4;

				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR MESSIER / ESCRIBIR CATID / VOLVER (Identificador de menú = 4)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 44:

			case 45:	// Estamos en el MENU PRINCIPAL / GOTO / SELECCIONAR POR NOMBRE / ESCRIBIR NOMBRE. 
				previousMenuSelection = 4;

				// Compruebo el comando que se ha elegido.
				switch(cmd) {
					case '0': // Se eligió MENU PRINCIPAL / GOTO / SELECCIONAR POR NOMBRE / ESCRIBIR NOMBRE / VOLVER (Identificador de menú = 4)
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 45:

			case 7: // Estamos en MENU PRINCIPAL / MOTORES.
				previousMenuSelection = 0;

				switch(cmd) {
					case '1':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Habilitando motor AR."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_AR_MOTOR_DRIVER;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();									
					break;

					case '2':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("DEShabilitando motor AR."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_AR_MOTOR_DRIVER;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;

					case '3':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Habilitando motor DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_DEC_MOTOR_DRIVER;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;

					case '4':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("DEShabilitando motor DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_DEC_MOTOR_DRIVER;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;

					case '5':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Habilitando motores AR y DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_ENABLE_AR_DEC_MOTOR_DRIVERS;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;

					case '6':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("DEShabilitando motores AR y DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DISABLE_AR_DEC_MOTOR_DRIVERS;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;

					case '7':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Realizando vuelta completa al eje de AR."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_AXIS_FULL_REVOLUTION;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();					
					break;

					case '8':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Realizando vuelta completa al eje de DEC."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_AXIS_FULL_REVOLUTION;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;


					case '9':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Parando motores."));

						// Convierto la pulsación de teclas en un comando para el telescopio.
        				sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_STOP;

						// Enviamos comando.
        				myTeles.sendData();
	
						printMenu();
					break;

					case 'a':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Información de consumo eléctrico."));
	
						printMenu();

						printCurrent();
					break;

					case '0':
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 7
			
			case 15: // Estamos en // MENU PRINCIPAL /PONER EN ESTACIÓN / ESTRELLA DE REFERENCIA.
				previousMenuSelection = 1;

				switch(cmd) {
					case '1':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Calculando AR para punto cardinal SUR"));
					
						// Muestro lista de estrellas (TOP 10) para el punto cardinal solicitado.
						myDB.selectStarsNearCardinalPoint(
							myAstro.computeARFromCardinalPoint(ToteAstro::cardinalPointEnum::CARDINAL_POINT_SOUTH), 
							2.0, 										// Límite de magnitud.
							myAstro.getObserverLatitude(),				// La latitud del observador.
							9);										// Límite de registros devueltos. No se puede pasar el máximo definido en MAX_LIMIT definida en la clase.

						// Fuerzo a ir a un menú especial, el 159, donde listo el recordset y obligo a elegir una estrella de referencia.
						menuSelection = 159;
						printMenu();
					break;

					case '2':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Calculando AR para punto cardinal OESTE"));
							
						// Muestro lista de estrellas (TOP 10) para el punto cardinal solicitado.
						myDB.selectStarsNearCardinalPoint(
							myAstro.computeARFromCardinalPoint(ToteAstro::cardinalPointEnum::CARDINAL_POINT_WEST), 
							2.0, 										// Límite de magnitud.
							myAstro.getObserverLatitude(),				// La latitud del observador.
							9);										// Límite de registros devueltos. No se puede pasar el máximo definido en MAX_LIMIT definida en la clase.

						// Fuerzo a ir a un menú especial, el 159, donde listo el recordset y obligo a elegir una estrella de referencia.
						menuSelection = 159;
						printMenu();
					break;

					case '3':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Calculando AR para punto cardinal NORTE"));
						// Muestro lista de estrellas (TOP 10) para el punto cardinal solicitado.
						myDB.selectStarsNearCardinalPoint(
							myAstro.computeARFromCardinalPoint(ToteAstro::cardinalPointEnum::CARDINAL_POINT_NORTH), 
							2.0, 										// Límite de magnitud.
							myAstro.getObserverLatitude(),				// La latitud del observador.
							9);										// Límite de registros devueltos. No se puede pasar el máximo definido en MAX_LIMIT definida en la clase.
						// Fuerzo a ir a un menú especial, el 159, donde listo el recordset y obligo a elegir una estrella de referencia.
						menuSelection = 159;
						printMenu();
					break;

					case '4':
						_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("Calculando AR para punto cardinal ESTE"));
							// Muestro lista de estrellas (TOP 10) para el punto cardinal solicitado.
						myDB.selectStarsNearCardinalPoint(
							myAstro.computeARFromCardinalPoint(ToteAstro::cardinalPointEnum::CARDINAL_POINT_EAST), 
							2.0, 										// Límite de magnitud.
							myAstro.getObserverLatitude(),				// La latitud del observador.
							9);										// Límite de registros devueltos. No se puede pasar el máximo definido en MAX_LIMIT definida en la clase.
						// Fuerzo a ir a un menú especial, el 159, donde listo el recordset y obligo a elegir una estrella de referencia.
						menuSelection = 159;
						printMenu();
					break;

					case '0':
						menuSelection = previousMenuSelection;
						printMenu();
					break;
				}
			break; // De case 15

			case 159: // Menú especial. Muestra un recordset (hasta 9) para elegir estrella de referencia.
				previousMenuSelection = 15;

				// Captura índice de la extrella.
				uint8_t subCmd = data.toInt();

				// Compruebo si se ha pulsado '0' (volver a menú anterior)
				if (subCmd == 0) {
					menuSelection = previousMenuSelection;
					printMenu();

					break;
				}
			
				// Compruebo si hay recordset.
				if (myDB.getNumberOfRows() == 0 || subCmd > myDB.getNumberOfRows() ) {
					// No hay ninguno o el valor elegido está fuera de índice.
					_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("processMenu()"), F("No hay registros en el recodset o el índice elegido supera el máximo."));

					printMenu();
					break;
				}

				// Establezco como objeto de referencia el registro 'subCmd' del recordset.
				myAstro.setReferenceObject(myDB.getRow(subCmd));	

				printMenu();	
			break; // De case 159
		} // De switch(menuSelection)
	}  // De if (Serial.available()) 

    
}

void printMenu(void) {
	Serial.println();
	Serial.println();

	switch (menuSelection) {
		case 0: // Menú principal.
			Serial.println(F("MENU PRINCIPAL"));
			Serial.println(F("--------------"));
			Serial.println();
			Serial.println(F("1..... Poner en estación."));
			Serial.println(F("2..... Sincronizar AR."));
			Serial.println(F("3..... Sincronizar DEC."));
			Serial.println(F("4..... Hacer Goto."));
			Serial.println(F("5..... Calcular tránsito."));
			Serial.println(F("6..... Ir a origen."));
			Serial.println(F("7..... Motores."));
		break;

		case 1: // MENU PRINCIPAL / PONER EN ESTACIÓN
			Serial.println(F("PONER EN ESTACIÓN"));
			Serial.println(F("-----------------"));
			Serial.println();
			Serial.println(F("1..... Fecha."));
			Serial.println(F("2..... Hora."));
			Serial.println(F("3..... Latitud."));
			Serial.println(F("4..... Longitud."));
			Serial.println(F("5..... Estrella de referencia."));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 15:  // MENU PRINCIPAL /PONER EN ESTACIÓN / ESTRELLA DE REFERENCIA /
			Serial.println(F("ESTRELLA DE REFERENCIA"));
			Serial.println(F("----------------------"));
			Serial.println();
			Serial.println(F("1..... SUR."));
			Serial.println(F("2..... OESTE."));
			Serial.println(F("3..... NORTE."));
			Serial.println(F("4..... ESTE."));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;


		case 159: // Menú especial donde se elige una estrella del recordset.
			Serial.println(F("SELECCION DE ESTRELLA DE REFERENCIA"));
			Serial.println(F("-----------------------------------"));
			for (int i = 1 ; i <= myDB.getNumberOfRows(); i++) {
        		Serial.println();
        		Serial.print(i); Serial.print(".....");
        		Serial.print(" "); Serial.print(F(" name=")); Serial.print(myDB.getRow(i).name);
        		Serial.print(" "); Serial.print(F(" ra=")); Serial.print(myDB.getRow(i).ra);
        		Serial.print(" "); Serial.print(F(" dec=")); Serial.print(myDB.getRow(i).dec);
        		Serial.print(" "); Serial.print(F(" mag=")); Serial.print(myDB.getRow(i).mag);
        		Serial.print(" "); Serial.print(F(" bayer=")); Serial.print(myDB.getRow(i).bayer);
        		Serial.print(" "); Serial.print(F(" flam=")); Serial.print(myDB.getRow(i).flam);
        		Serial.print(" "); Serial.print(F(" cons=")); Serial.print(myDB.getRow(i).cons);
        		Serial.print(" "); Serial.print(F(" cat=")); Serial.print(myDB.getRow(i).cat);
        		Serial.print(" "); Serial.print(F(" catId=")); Serial.print(myDB.getRow(i).catId);
    		}

			Serial.println();
			Serial.println(F("0..... Volver."));
		break; // De case 159:



		case 2: // MENU PRINCIPAL / SINCRONIZAR AR
			Serial.println(F("SINCRONIZAR AR"));
			Serial.println(F("--------------"));
			Serial.println();
			Serial.println(F("1..... +AR."));
			Serial.println(F("2..... -AR."));
			Serial.println(F("3..... Roster V AR/DEC."));
			Serial.println(F("4..... Stop."));
			Serial.println(F("5..... Sincronizar AR."));
			Serial.println(F("6..... Tracking ON/OFF."));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 3: // MENU PRINCIPAL / SINCRONIZAR DEC
			Serial.println(F("SINCRONIZAR DEC"));
			Serial.println(F("--------------"));
			Serial.println();
			Serial.println(F("1..... +DEC."));
			Serial.println(F("2..... -DEC."));
			Serial.println(F("3..... Roster V AR/DEC."));
			Serial.println(F("4..... Stop."));
			Serial.println(F("5..... Sincronizar DEC."));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 4: // MENU PRINCIPAL / GOTO.
			Serial.println(F("GOTO"));
			Serial.println(F("----"));
			Serial.println();
			Serial.println(F("1..... Seleccionar estrella BAYER."));
			Serial.println(F("2..... Seleccionar estrella FLAMSTEED."));
			Serial.println(F("3..... Seleccionar objeto NGC."));
			Serial.println(F("4..... Seleccionar objeto Messier."));
			Serial.println(F("5..... Seleccionar objeto por nombre."));
			Serial.println(F("6..... Hacer Goto."));
			Serial.println(F("7..... Cancelar Goto / Stop."));
			Serial.println(F("8..... Tracking ON/OFF."));
			Serial.println(F("9..... Goto Object --> Ref."));
			Serial.println(F("a..... AR/DEC del telescopio."));
			Serial.println(F("b..... Calcular giro Tierra ON/OFF."));
			
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 41: // MENU PRINCIPAL / GOTO / SELECCIONAR BAYER
			Serial.println(F("SELECCIONAR ESTRELLA CATÁLOGO BAYER"));
			Serial.println(F("-----------------------------------"));
			Serial.println();
			Serial.println(F("1..... Seleccionar CONSTELACIÓN."));
			Serial.println(F("2..... Seleccionar LETRA."));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 411: // MENU PRINCIPAL / GOTO / SELECCIONAR BAYER / CONSTELACIÓN
			Serial.println(F("ESCRIBE CONSTELACIÓN"));
			Serial.println(F("--------------------"));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 412: // MENU PRINCIPAL / GOTO / SELECCIONAR BAYER / LETRA
			Serial.println(F("ESCRIBE LETRA"));
			Serial.println(F("-------------"));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 42: // MENU PRINCIPAL / GOTO / SELECCIONAR FLAMSTEED
			Serial.println(F("SELECCIONAR ESTRELLA CATÁLOGO FLAMSTEED"));
			Serial.println(F("---------------------------------------"));
			Serial.println();
			Serial.println(F("1..... Seleccionar CONSTELACIÓN."));
			Serial.println(F("2..... Seleccionar NÚMERO."));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 421: // MENU PRINCIPAL / GOTO / SELECCIONAR FLAMSTEED / CONSTELACIÓN
			Serial.println(F("ESCRIBE CONSTELACIÓN"));
			Serial.println(F("--------------------"));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 422: // MENU PRINCIPAL / GOTO / SELECCIONAR FLAMSTEED / NÚMERO
			Serial.println(F("ESCRIBE NÚMERO"));
			Serial.println(F("-------------"));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 43: // MENU PRINCIPAL / GOTO / SELECCIONAR NGC / CATID
			Serial.println(F("ESCRIBE CATID"));
			Serial.println(F("-------------"));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 44: // MENU PRINCIPAL / GOTO / SELECCIONAR MESSIER / CATID
			Serial.println(F("ESCRIBE CATID"));
			Serial.println(F("----------"));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		case 45: // MENU PRINCIPAL / GOTO / SELECCIONAR POR NOMBRE / NOMBRE
			Serial.println(F("ESCRIBE NOMBRE"));
			Serial.println(F("--------------"));
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		
		case 7:  // MENU PRINCIPAL / MOTORES
			Serial.println(F("MOTORES"));
			Serial.println(F("-------"));
			Serial.println();
			Serial.println(F("1..... Habilitar motor AR."));
			Serial.println(F("2..... DEShabilitar motor AR."));
			Serial.println(F("3..... Habilitar motor DEC."));
			Serial.println(F("4..... DEShabilitar motor DEC."));
			Serial.println(F("5..... Habilitar motores AR/DEC."));
			Serial.println(F("6..... DEShabilitar motores AR/DEC."));
			Serial.println(F("7..... Vuelta completa eje AR."));
			Serial.println(F("8..... Vuelta completa eje DEC."));
			Serial.println(F("9..... Detener motores."));
			Serial.println(F("a..... Consumo eléctrico."));
			
			Serial.println();
			Serial.println(F("0..... Volver."));
		break;

		default:
			Serial.println(F("NO IMPLEMENTADO. VOLVIENDO A MENU PRINCIPAL."));
			menuSelection = 0;
			printMenu();
		break;
	}

	// Muestro información de estado.
	Serial.println();

	Serial.println(F("Pulsa tecla 'M' para refrescar información"));

	// Objeto de referencia.
	if (myAstro.getIsReferenceObjectSet()) {
		Serial.print(F("Estrella referencia: "));
		Serial.print(F(" name=")); Serial.print(myAstro.getReferenceObject().name);
        Serial.print(F(" ra=")); Serial.print(myAstro.getReferenceObject().ra);
        Serial.print(F(" dec=")); Serial.print(myAstro.getReferenceObject().dec);
        Serial.print(F(" mag=")); Serial.print(myAstro.getReferenceObject().mag);
        Serial.print(F(" bayer=")); Serial.print(myAstro.getReferenceObject().bayer);
        Serial.print(F(" flam=")); Serial.print(myAstro.getReferenceObject().flam);
        Serial.print(F(" cons=")); Serial.print(myAstro.getReferenceObject().cons);
        Serial.print(F(" cat=")); Serial.print(myAstro.getReferenceObject().cat);
        Serial.print(F(" catId=")); Serial.println(myAstro.getReferenceObject().catId);
	} else {
		Serial.println(F("Estrella refencia no elegida."));
	}

	// Objeto para hacer Goto.
	if (myAstro.getIsGotoObjectSet()) {
		Serial.print(F("Goto Object        : "));
		Serial.print(F(" name=")); Serial.print(myAstro.getGotoObject().name);
        Serial.print(F(" ra=")); Serial.print(myAstro.getGotoObject().ra);
        Serial.print(F(" dec=")); Serial.print(myAstro.getGotoObject().dec);
        Serial.print(F(" mag=")); Serial.print(myAstro.getGotoObject().mag);
        Serial.print(F(" bayer=")); Serial.print(myAstro.getGotoObject().bayer);
        Serial.print(F(" flam=")); Serial.print(myAstro.getGotoObject().flam);
        Serial.print(F(" cons=")); Serial.print(myAstro.getGotoObject().cons);
        Serial.print(F(" cat=")); Serial.print(myAstro.getGotoObject().cat);
        Serial.print(F(" catId=")); Serial.println(myAstro.getGotoObject().catId);
	} else {
		Serial.println(F("Goto object no elegido."));
	}

	// Información de tracking.
	Serial.print("TRACKING  : ");
	Serial.println(isARTracking);

	// Información del motor AR.
	Serial.print(F("AR MOTOR : "));
	Serial.print(F("isRunning: "));
	Serial.print(isARMotorRunning);
	Serial.print(F(", isSynced: "));
	Serial.println(isARMotorSynced);


	// Información del motor DEC.
	Serial.print(F("DEC MOTOR: "));
	Serial.print(F("isRunning: "));
	Serial.print(isDECMotorRunning);
	Serial.print(F(", isSynced: "));
	Serial.println(isDECMotorSynced);

	Serial.print(F("AR/DEC VmIdx: "));
	Serial.println(ARDECMotorsVmArrayIndex);

	printCurrent();

	Serial.println();

	if (isComputeAries) {
		Serial.println(F("Se tendrá en cuenta el giro de la Tierra para hacer el Goto."));
	} else {
		Serial.println(F("NO se tendrá en cuenta el giro de la Tierra para hacer el Goto."));
	}
	
	printARDECTeles();
	
	Serial.println();
}

void printCurrent(void) {
	// Si la batería está por debajo de 11V. Muestro advertencia.
	if (ina3221Data[0].busVoltage < 11.0) {
		Serial.println();
		Serial.println(F("***************************************"));
		Serial.println(F("***************************************"));
		Serial.println(F("   ATENCIÓN, VOLTAJE DE BATERÍA BAJO"));
		Serial.println(F("***************************************"));
		Serial.println(F("***************************************"));
	}

	Serial.println();
	Serial.print(F("vBattery: "));
	Serial.println(ina3221Data[0].busVoltage);
	Serial.print(F("AR Current: "));
	Serial.println(ina3221Data[0].current_mA);
	Serial.print(F("DEC Current: "));
	Serial.println(ina3221Data[1].current_mA);
	Serial.print(F("REST Current: "));
	Serial.println(ina3221Data[2].current_mA);
}

void printARDECTeles(void) {
	// Información de AR y DEC del telescopio.
	Serial.print(F("AR Telescopio  : "));
	Serial.println(myAstro.getTelesAR());
	Serial.print(F("DEC Telescopio : "));
	Serial.println(myAstro.getTelesDEC());
}

// Envía latido al slave.
void sendHeartbeat(void){
	// Ojo, 'sendigData' está declarada 'extern' en ToteTelesCtrlESPNOW_VX_X.cpp
	
	// Compruebo si es el momento.
	if (millis() - lastHBMillis > HEARTBEAT_PERIOD) {
		// Convierto la pulsación de teclas en un comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_HEARTBEAT_REQUEST;

        // Enviamos comando.
        myTeles.sendData();

		_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_(F("sendHeartbeat()"), F("Comando enviado: "), myTeles.getCtrlMsgStr(sendingData.cmdRes));
		
		// Preparamos temporizador para próximo evento.
		lastHBMillis = millis();
	}
}

// Pedimos muestro de consumo de corriente de motores.
void sendINA3221Sense(void){
	// Ojo, 'sendigData' está declarada 'extern' en ToteTelesCtrlESPNOW_VX_X.cpp
	
	// Compruebo si es el momento.
	if (millis() - lastINASenseMillis > INA3221_SENSE_PERIOD) {
		// Convierto la pulsación de teclas en un comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_INA3221_SENSE;

        // Enviamos comando.
        myTeles.sendData();

		_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_(F("sendHeartbeat()"), F("Comando enviado: "), myTeles.getCtrlMsgStr(sendingData.cmdRes));
		
		// Preparamos temporizador para próximo evento.
		lastINASenseMillis = millis();
	}
}

void sendARDECRequest(void) {	// Ojo, 'sendigData' está declarada 'extern' en ToteTelesCtrlESPNOW_VX_X.cpp
	// Compruebo si es el momento.
	if (millis() - lastARDECRequestMillis > AR_DEC_REQUEST_PERIOD) {
		// Convierto la pulsación de teclas en un comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_REQUEST;

        // Enviamos comando.
        myTeles.sendData();

		_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_(F("sendHeartbeat()"), F("Comando enviado: "), myTeles.getCtrlMsgStr(sendingData.cmdRes));
		
		// Preparamos temporizador para próximo evento.
		lastARDECRequestMillis = millis();
	}
}

// Solicita información de movimiento de los motores.
void getStatusInfo(void) {
	// Ojo, 'sendigData' está declarada 'extern' en ToteTelesCtrlESPNOW_VX_X.cpp
	
	// Compruebo si es el momento.
	if (millis() - lastGSIMillis > GET_STATUS_INFO_PERIOD) {
		// Cargo comando para motor AR.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GET_STATUS_INFO;

        // Enviamos comando.
        myTeles.sendData();

		_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_(F("getStatusInfo()"), F("Comando enviado: "), myTeles.getCtrlMsgStr(sendingData.cmdRes));
		
		// Preparamos temporizador para próximo evento.
		lastGSIMillis = millis();
	}
}

 // Compruebo si hay mensajes por atender.
void checkNewMsg(void) {
	if (myTeles.checkNewMsg()) {
		switch(receivedDataPtr->cmdRes) { // cmdRes tiene el comando recibido.
			case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_GET_STATUS_INFO_DATA:
				isARMotorRunning 		= receivedDataPtr->isARMotorRunning;		// Semáforo para indicar si se está moviendo el motor de AR.
				isARMotorSynced 		= receivedDataPtr->isARMotorSynced;			// Semáforo para indicar si se AR está sincronizado.
				isDECMotorRunning 		= receivedDataPtr->isDECMotorRunning;		// Semáforo para indicar si se está moviendo el motor de AR.
				isDECMotorSynced 		= receivedDataPtr->isDECMotorSynced;		// Semáforo para indicar si se DEC está sincronizado.
				isARTracking			= receivedDataPtr->isARTracking;			// Semáforo para indicar si se está haciendo el tracking.
				ARDECMotorsVmArrayIndex	= receivedDataPtr->ARDECMotorsVmArrayIndex; // Almacena índice del roster de velocidad para AR/DEC.

				_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_(F("checkNewMsg()"), F("Comando recibido: "), myTeles.getCtrlMsgStr(receivedDataPtr->cmdRes));
		
			break;

			case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_REQUEST_ACK:
				myAstro.setTelesAR(receivedDataPtr->ra);			// Almaceno la AR devuelta.
				myAstro.setTelesDEC(receivedDataPtr->dec);			// Almaceno la DEC devuelta.
				
				_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(F("checkNewMsg()"), F("telesAR: "), myAstro.getTelesAR());
				_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(F("checkNewMsg()"), F("telesDEC: "), myAstro.getTelesDEC());
			break;

			case ToteTelesCtrlESPNOW::TELES_CTRL_MSG_INA3221_SENSE_ACK:
				ina3221Data[0].busVoltage = receivedDataPtr->vBattery;  	// El canal 1 (idx 0) almacena la tensión de la batería.
				ina3221Data[0].current_mA = receivedDataPtr->AR_mA;			// El canal 1 (idx 0) almacena la corriente del motor AR.
				ina3221Data[1].current_mA = receivedDataPtr->DEC_mA;		// El canal 2 (idx 1) almacena la corriente del motor DEC.
				ina3221Data[2].current_mA = receivedDataPtr->REST_mA;		// El canal 3 (idx 2) almacena la corriente del resto de la electrónica.

				_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(F("checkNewMsg()"), F("vBattery: "), ina3221Data[0].busVoltage);
				_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(F("checkNewMsg()"), F("AR Current: "), ina3221Data[0].current_mA);
				_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(F("checkNewMsg()"), F("DEC Current: "), ina3221Data[1].current_mA);
				_GOTO_CONTROLLER_LEVEL_2_DEBUG_VALUE_DOUBLE_(F("checkNewMsg()"), F("REST Current: "), ina3221Data[2].current_mA);
			break;
		}
	}  
}

// Lee el joystick y envía comando de movimiento a motores.
void sendJoystickCmd(void) {
	bool isJoystickMoving = false;	// En principio supongo que no se está usando el joystick.

	// Si no se ha cumplido aún el intervalo de muestreo, salgo.
	if (!myJoystick.check()) {
		return;
	}

	// Estamos dentro del intervalo de muestreo.


	// Comando AR INC y aún no se ha enviado comando de movimiento.
	if (myJoystick.getARINC()) {
		_GOTO_CONTROLLER_LEVEL_2_DEBUG_(F("sendJoystickCmd()"), F("Posicionamiento manual por Joystick (INCREMENTAR AR)"));

		// Comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP;

        // Enviamos comando.
        myTeles.sendData();

		// Indico que el Joystick se está usando.
		isJoystickMoving = true;

		// Preparamos para la siguiente vez.
		isStopCmdSent = false;
	} 

	// Comando AR DEC y aún no se ha enviado comando de movimiento.
	if (myJoystick.getARDEC()) {
		_GOTO_CONTROLLER_LEVEL_2_DEBUG_(F("sendJoystickCmd()"), F("Posicionamiento manual por Joystick (DECREMENTAR AR)"));

		// Comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP;

        // Enviamos comando.
        myTeles.sendData();

		// Indico que el Joystick se está usando.
		isJoystickMoving = true;

		// Preparamos para la siguiente vez.
		isStopCmdSent = false;
	} 

	// Comando DEC INC y aún no se ha enviado comando de movimiento.
	if (myJoystick.getDECINC()) {
		_GOTO_CONTROLLER_LEVEL_2_DEBUG_(F("sendJoystickCmd()"), F("Posicionamiento manual por Joystick (INCREMENTAR DEC)"));

		// Comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_INC_DELTA_STEP;

        // Enviamos comando.
        myTeles.sendData();

		// Indico que el Joystick se está usando.
		isJoystickMoving = true;

		// Preparamos para la siguiente vez.
		isStopCmdSent = false;
	} 

	// Comando DEC DEC y aún no se ha enviado comando de movimiento.
	if (myJoystick.getDECDEC()) {
		_GOTO_CONTROLLER_LEVEL_2_DEBUG_(F("sendJoystickCmd()"), F("Posicionamiento manual por Joystick (DECREMENTAR DEC)"));

		// Comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_DEC_MOTOR_MANUAL_POSITIONING_DEC_DELTA_STEP;

        // Enviamos comando.
        myTeles.sendData();


		// Indico que el Joystick se está usando.
		isJoystickMoving = true;

		// Preparamos para la siguiente vez.
		isStopCmdSent = false;
	} 

	// Comando SW y aún no se ha enviado comando de movimiento.
	if (myJoystick.getSW()) {
		_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendJoystickCmd()"), F("Posicionamiento manual por Joystick (ROSTER VELOCIDAD)"));

		// Comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTORS_NEXT_ROSTER_VM;

        // Enviamos comando.
      	myTeles.sendData();
	}  

	// Si no se está usando el joystick y tampoco se ha enviado el comando stop, detengo los motores.
	if (!isJoystickMoving && !isStopCmdSent) {
		_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendJoystickCmd()"), F("Deteniendo motores AR y DEC."));

		// Comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_MOTOR_STOP;

        // Enviamos comando.
        myTeles.sendData();

		// Indico que ya se envió el comando.
		isStopCmdSent = true;

		// Indico que el Joystick se está usando.
		isJoystickMoving = false;
	}  
}	

// Leeo botones y envío comandos.
void sendButtonsCmd(void) {
	// Compruebo botón de sincronización de AR y DEC.
	if (mySincroARDECBtn.check()) {
		_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendButtonsCmd()"), F("Pulsado botón sincronización AR/DEC."));

		// Compruebo si hay estrella de referencia elegida.
		if (!myAstro.getIsReferenceObjectSet()) {
			_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendButtonsCmd()"), F("No se ha elegido estrella de referencia. No se puede sincronizar."));
		} else {
			// Comando para el telescopio.
        	sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_AR_DEC_SYNC;

			// Agrego información a la estructura.
			sendingData.utc = myAstro.getUTC();
			sendingData.ra =  myAstro.getReferenceObject().ra;
			sendingData.dec = myAstro.getReferenceObject().dec;

    	    // Enviamos comando.
        	myTeles.sendData();
		}
	}

	// Compruebo botón de encendido apagado de láser.
	if (myLaserONOFFBtn.check()) {
		// Compruebo si estaba encendido.
		if (laserOnOffToogle) {
			// Lo apago.
		    sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_LASER_OFF;

			_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendButtonsCmd()"), F("Apagando Láser."));
	
			// Cambio es estado.
			laserOnOffToogle = false;
		} else {
			// Lo enciendo.
			 sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_LASER_ON;

			 _GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendButtonsCmd()"), F("Encenciendo Láser."));

			 // Cambio el estado.
			 laserOnOffToogle = true;
		}

		// Enviamos comando.
        myTeles.sendData();
	}

	// Compruebo botón de Tracking..
	if (myTrackingONOFFBtn.check()) {
		_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendButtonsCmd()"), F("Pulsado botón Tracking ON/OFF."));
	
		// Comando para el telescopio.
        sendingData.cmdRes = ToteTelesCtrlESPNOW::TELES_CTRL_MSG_TRACKING_ON_OFF;

		// Enviamos comando.
        myTeles.sendData();
	}

	// Compruebo botón de swap Goto ref
	if (mySwapGotoRefBtn.check()) {
		_GOTO_CONTROLLER_LEVEL_1_DEBUG_(F("sendButtonsCmd()"), F("Pulsado botón Swap goto-->Ref."));
	
		// intercambio.
		swapGotoRef();
	}
}

// intercambio los objetos ref y goto.
void swapGotoRef(void) {
	myAstro.swapGotoRef();

	printMenu();		
}